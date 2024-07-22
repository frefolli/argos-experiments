#ifndef TASK_EXECUTOR_DEFAULT_HH
#define TASK_EXECUTOR_DEFAULT_HH
/** @file default.hh */
#include "support/coordination.hh"
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/vector3.h>
#include <iostream>
#include <support/task_executor.hh>
#include <support/rab.hh>
#include <support/targets.hh>
#include <support/task.hh>
#include <support/setup.hh>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_speed_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <cstdint>

namespace prez::task_executors
{
  class Default : public TaskExecutor
  {
    static constexpr uint32_t MAX_WAITED_ROUNDS = 20;

    static constexpr argos::Real IN_METERS = 10e-3;
    static constexpr argos::Real TARGET_DISTANCE = 2.0f;
    static constexpr argos::Real MAX_INTERACTION = 20.0f;
    static constexpr double_t MAX_COLLISION_AVOIDANCE_DISTANCE = 7.0f;

    static constexpr argos::Real GP_ACCEL = 4.0f;
    static constexpr argos::Real JP_ACCEL = 16.0f;
    static constexpr argos::Real LP_ACCEL = 0.2f;

    static constexpr argos::Real STEADY_LIMIT = 10e-4;

    enum CollisionAvoidancePotential
    {
      GP, /*Inspired by Gravitational Potential with ABS*/
      JP, /*Inspired by Gravitational Potential without ABS*/
      LP  /*Lennard Jones Potential*/
    } collision_avoidance_potential = GP;

    enum MotionAppliance
    {
      NOISELESS, /*doesn't apply noise to SetLinearVelocity*/
      NOISY,     /*applies noise to SetLinearVelocity*/
    } motion_appliance = NOISELESS;

    enum TakeOffStrategy
    {
      VERTICAL, /*first a vertical take off, then a direct flight*/
      DIRECT,   /*take off uses flight engine to gain elevation*/
    } take_off_strategy = VERTICAL;

  public:
    /* attributes read also by spiri_controller
     */
    enum State
    {
      START,
      AT_GROUND,
      TAKING_OFF,
      TAKEN_OFF,
      ARRIVED
    } state = START;

    uint32_t tick = 0;
    uint32_t waited_rounds = 0;
    uint32_t max_iterations = 0;

    Task *task = nullptr;
    argos::CRandom::CRNG *random_number_generator = nullptr;
    argos::CCI_PositioningSensor *positioning_sensor = nullptr;
    argos::CCI_RangeAndBearingSensor *range_and_bearing_sensor = nullptr;
    argos::CCI_QuadRotorSpeedActuator *speed_actuator = nullptr;

    argos::CVector3 last_position = {-1,-1,-1};
    double_t delta_position = 0;
    bool already_sprinted = false;

    inline void ParseCollisionAvoidancePotential()
    {
      collision_avoidance_potential = GP;
      char *strategy = std::getenv("COLLISION_AVOIDANCE_POTENTIAL");
      if (strategy != nullptr)
      {
        PARSE_ENV_SETUP(collision_avoidance_potential, GP);
        PARSE_ENV_SETUP(collision_avoidance_potential, JP);
        PARSE_ENV_SETUP(collision_avoidance_potential, LP);
      }
      // std::cout << "Using COLLISION_AVOIDANCE_POTENTIAL = " << collision_avoidance_potential << std::endl;
    }

    inline void ParseMotionAppliance()
    {
      motion_appliance = NOISELESS;
      char *strategy = std::getenv("MOTION_APPLIANCE");
      if (strategy != nullptr)
      {
        PARSE_ENV_SETUP(motion_appliance, NOISELESS);
        PARSE_ENV_SETUP(motion_appliance, NOISY);
      }
      // std::cout << "Using MOTION_APPLIANCE = " << motion_appliance << std::endl;
    }

    inline void ParseMaxIteration()
    {
      char *strategy = std::getenv("MAX_ITERATION");
      if (strategy != NULL)
        max_iterations = atoi(strategy);

      // std::cout << "Using max_iterations = " << max_iterations << std::endl;
    }

    inline void ParseTakeOffStrategy()
    {
      take_off_strategy = VERTICAL;
      char *strategy = std::getenv("TAKE_OFF_STRATEGY");
      if (strategy != nullptr)
      {
        PARSE_ENV_SETUP(take_off_strategy, VERTICAL);
        PARSE_ENV_SETUP(take_off_strategy, DIRECT);
      }
      // std::cout << "Using TAKE_OFF_STRATEGY = " << take_off_strategy << std::endl;
    }

    void Init()
    {
      ParseCollisionAvoidancePotential();
      ParseMotionAppliance();
      ParseMaxIteration();
      ParseTakeOffStrategy();
    }

    void Start()
    {
      ComputeHeadingToTarget();
      ++waited_rounds;
      if (waited_rounds > MAX_WAITED_ROUNDS)
      {
        state = State::AT_GROUND;
      }
      else
      {
        ++waited_rounds;
      }
    }

    void AtGround()
    {
      ComputeHeadingToTarget();
      uint32_t waiting_queue = 0;
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours)
      {
        argos::Real range = neighbour.Range * IN_METERS;
        if (range < MAX_COLLISION_AVOIDANCE_DISTANCE)
        {
          if (neighbour.Data[prez::RABKey::ID] > task->id
           && (neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::AT_GROUND
           || neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::TAKING_OFF))
          {
            waiting_queue++;
          }
        }
      }

      if (waiting_queue == 0)
      {
        switch (take_off_strategy) {
          case TakeOffStrategy::VERTICAL: {
            SetLinearVelocity({0.0f, 0.0f, 10.0f});
            state = State::TAKING_OFF;
          }; break; 
          case TakeOffStrategy::DIRECT: {
            SetLinearVelocity({0.0f, 0.0f, 10.0f});
            state = State::TAKEN_OFF;
          }; break; 
        }
      }
    }

    void TakingOff()
    {
      ComputeHeadingToTarget();
      // If i left 2.0f meters above ground i can consider myself taken off
      // I set and maintain current heading target
      argos::CVector3 current_position = positioning_sensor->GetReading().Position;
      if (current_position.GetZ() >= 1.5f)
      {
        SetLinearVelocity({0.0f, 0.0f, 0.0f});
        state = State::TAKEN_OFF;
        last_position = current_position;
      }
    }

    argos::CVector3 ApproachTarget()
    {
      ComputeHeadingToTarget();
      if (task->target_direction.Length() > MAX_INTERACTION)
      {
        argos::CVector3 direction = task->target_direction.Normalize() * MAX_INTERACTION;
        return direction;
      }
      return task->target_direction;
    }

    argos::Real ApplyCollisionAvoidancePotential(argos::Real distance)
    {
      switch (collision_avoidance_potential)
      {
      case GP:
      {
        argos::Real difference = TARGET_DISTANCE - distance;
        return -GP_ACCEL * ::abs(difference) / distance;
      };
      case JP:
      {
        argos::Real difference = TARGET_DISTANCE - distance;
        return -JP_ACCEL * difference / (distance * distance);
      };
      case LP:
      {
        argos::Real B = ::pow(TARGET_DISTANCE / distance, 6);
        argos::Real A = B * B;
        return (B - A) * LP_ACCEL * 4;
      };
      default:
        return 0.0f;
      }
    }

    argos::CVector3 AvoidObstacles()
    {
      argos::CVector2 direction(0.0f, 0.0f);
      argos::CQuaternion orientation = positioning_sensor->GetReading().Orientation;
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      uint32_t n_of_forces = 0;
      for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours)
      {
        argos::Real range = neighbour.Range * IN_METERS;
        if ((neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::TAKING_OFF || neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::TAKEN_OFF || neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::ARRIVED) && range < MAX_COLLISION_AVOIDANCE_DISTANCE)
        {
          argos::CVector2 avoidance(ApplyCollisionAvoidancePotential(range), neighbour.HorizontalBearing);
          n_of_forces++;
          direction += avoidance;
        }
      }
      if (n_of_forces > 0)
      {
        direction = direction / n_of_forces;
        argos::CVector3 avoidance = argos::CVector3(direction.GetX(), direction.GetY(), 0.0f);
        avoidance = avoidance.Rotate(orientation);
        avoidance.SetZ(0.0f);
        if (avoidance.Length() > MAX_INTERACTION)
        {
          avoidance = avoidance.Normalize() * MAX_INTERACTION;
        }
        return avoidance;
      }
      return {0.0f, 0.0f, 0.0f};
    }

    void TakenOff()
    {
      argos::CVector3 direction(0.0f, 0.0f, 0.0f);
      direction += ApproachTarget();
      direction += AvoidObstacles();
      task->speed = direction.Length();
      SetLinearVelocity(direction);

      if (already_sprinted && delta_position < STEADY_LIMIT)
      {
        prez::Coordination::GetInstance().Finished();
        state = State::ARRIVED;
        argos::CVector3 stop(0.0f, 0.0f, 0.0f);
        SetLinearVelocity(stop);
        std::cout << task->id << " going down" << std::endl;
      }
    }

    void Arrived()
    {
      argos::CVector3 direction(0.0f, 0.0f, 0.0f);
      direction += ApproachTarget();
      direction += AvoidObstacles();
      task->speed = direction.Length();
      SetLinearVelocity(direction);
    }

    void Round()
    {
      tick++;
      if (tick == max_iterations)
      {
        prez::Coordination::GetInstance().Finished();
      }

      switch (state)
      {
      case State::START:
        Start();
        break;
      case State::AT_GROUND:
        AtGround();
        break;
      case State::TAKING_OFF:
        TakingOff();
        break;
      case State::TAKEN_OFF:
        TakenOff();
        break;
      case State::ARRIVED:
        Arrived();
        break;
      }
    }

    void Reset()
    {
      state = START;
      waited_rounds = 0;
    }

    void ComputeHeadingToTarget()
    {
      argos::CVector3 position = positioning_sensor->GetReading().Position;
      if (last_position != argos::CVector3(-1,-1,-1)) {
        delta_position = (position - last_position).Length();
      }
      if (!already_sprinted && delta_position > STEADY_LIMIT) {
        already_sprinted = true; 
      }
      last_position = position;
      argos::CVector3 &target_position = prez::GetTargetList()->at(task->target).position;
      argos::CVector3 direction = target_position - position;
      task->target_direction = direction;
      task->distance_from_target = direction.Length();
    }

    void SetLinearVelocity(argos::CVector3 motion)
    {
      switch (motion_appliance)
      {
      case NOISELESS:
      {
      };
      break;
      case NOISY:
      {
        motion.SetX(motion.GetX() + random_number_generator->Gaussian(1.0));
        motion.SetY(motion.GetY() + random_number_generator->Gaussian(1.0));
        motion.SetZ(motion.GetZ() + random_number_generator->Gaussian(1.0));
      };
      default:
      {
      }
      }
      speed_actuator->SetLinearVelocity(motion);
    }
  };
}
#endif // TASK_EXECUTOR_DEFAULT_HH
