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
#include <cstdint>

namespace prez::task_executors {
  class Default : public TaskExecutor {
    static constexpr uint32_t MAX_WAITED_ROUNDS = 20;

    static constexpr argos::Real IN_METERS = 10e-3;
    static constexpr argos::Real TARGET_DISTANCE = 2.0f;
    static constexpr argos::Real MAX_INTERACTION = 20.0f;
    static constexpr double_t MAX_COLLISION_AVOIDANCE_DISTANCE = 7.0f;
    
    static constexpr argos::Real GP_ACCEL = 1.0f;
    static constexpr argos::Real LP_ACCEL = 0.2f;

    enum CollisionAvoidancePotential {
      GP, LP
    } collision_avoidance_potential;

    public: 
    /* attributes read also by spiri_controller
    */
    enum State {
      START, AT_GROUND, TAKING_OFF, TAKEN_OFF, ARRIVED
    } state = START;

    uint32_t tick = 0;
    uint32_t waited_rounds = 0;

    Task* task;
    argos::CCI_PositioningSensor* positioning_sensor;
    argos::CCI_RangeAndBearingSensor* range_and_bearing_sensor;
    argos::CCI_QuadRotorSpeedActuator* speed_actuator;

    argos::CVector3 last_position;
    double_t delta_position;

    inline void ParseCollisionAvoidancePotential()
    {
      collision_avoidance_potential = GP;
      char *strategy = std::getenv("COLLISION_AVOIDANCE_POTENTIAL");
      if (strategy != nullptr)
      {
        PARSE_ENV_SETUP(collision_avoidance_potential, GP);
        PARSE_ENV_SETUP(collision_avoidance_potential, LP);
      }
      // std::cout << "Using COLLISION_AVOIDANCE_POTENTIAL = " << collision_avoidance_potential << std::endl;
    }

    void Init()
    {
      ParseCollisionAvoidancePotential();
    }

    void Start() {
      ComputeHeadingToTarget();
      ++waited_rounds;
      if (waited_rounds > MAX_WAITED_ROUNDS) {
        state = State::AT_GROUND;
      } else {
        ++waited_rounds;
      }
    }

    void AtGround() {
      ComputeHeadingToTarget();
      uint32_t waiting_queue = 0;
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
        argos::Real range = neighbour.Range * IN_METERS;
        if (range < MAX_COLLISION_AVOIDANCE_DISTANCE) {
          if (neighbour.Data[prez::RABKey::ID] > task->id && neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] != State::TAKEN_OFF) {
            waiting_queue++;
          }
        }
      }

      if (waiting_queue == 0) {
        // speed_actuator->SetRotationalSpeed(argos::CRadians::PI);
        speed_actuator->SetLinearVelocity({0.0f, 0.0f, 10.0f});
        state = State::TAKING_OFF;
      }
    }

    void TakingOff() {
      ComputeHeadingToTarget();
      // If i left 2.0f meters above ground i can consider myself taken off
      // I set and maintain current heading target
      argos::CVector3 current_position = positioning_sensor->GetReading().Position;
      if (current_position.GetZ() >= 1.5f) {
        speed_actuator->SetLinearVelocity({0.0f, 0.0f, 0.0f});
        state = State::TAKEN_OFF;
        last_position = current_position;
      }
    }

    argos::CVector3 ApproachTarget() {
      ComputeHeadingToTarget();
      if (task->target_direction.Length() > MAX_INTERACTION) {
        argos::CVector3 direction = task->target_direction.Normalize() * MAX_INTERACTION;
        return direction;
      }
      return task->target_direction;
    }

    argos::Real GravitationalPotential(argos::Real distance) {
      switch(collision_avoidance_potential) {
      case GP: {
        argos::Real difference = TARGET_DISTANCE - distance;
        return - GP_ACCEL * ::abs(difference) / distance;
      };
      case LP: {
        argos::Real B = ::pow(TARGET_DISTANCE / distance, 6);
        argos::Real A = B * B;
        return (B - A) * LP_ACCEL * 4;
      };
    }
    }

    argos::CVector3 AvoidObstacles() {
      argos::CVector2 direction(0.0f, 0.0f);
      argos::CQuaternion orientation = positioning_sensor->GetReading().Orientation;
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      uint32_t n_of_forces = 0;
      for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
        argos::Real range = neighbour.Range * IN_METERS;
        if ((neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::TAKING_OFF
          || neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::TAKEN_OFF
          || neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::ARRIVED)
          && range < MAX_COLLISION_AVOIDANCE_DISTANCE) {
          argos::CVector2 avoidance(GravitationalPotential(range), neighbour.HorizontalBearing);
          n_of_forces++;
          direction += avoidance;
        }
      }
      if (n_of_forces > 0) {
        direction = direction / n_of_forces;
        argos::CVector3 avoidance = argos::CVector3(direction.GetX(), direction.GetY(), 0.0f);
        avoidance = avoidance.Rotate(orientation);
        avoidance.SetZ(0.0f);
        if (avoidance.Length() > MAX_INTERACTION) {
          avoidance = avoidance.Normalize() * MAX_INTERACTION;
        }
        return avoidance;
      }
      return {0.0f, 0.0f, 0.0f};
    }

    void TakenOff() {
      argos::CVector3 direction(0.0f, 0.0f, 0.0f);
      direction += ApproachTarget();
      direction += AvoidObstacles();
      task->speed = direction.Length();
      speed_actuator->SetLinearVelocity(direction);

      if(delta_position < 10e-4) {
        prez::Coordination::GetInstance().Finished();
        state = State::ARRIVED;
        argos::CVector3 stop(0.0f, 0.0f, 0.0f);
        speed_actuator->SetLinearVelocity(stop);
        std::cout << task->id << " going down" << std::endl;
      }
    }

    void Arrived() {
      argos::CVector3 direction(0.0f, 0.0f, 0.0f);
      direction += ApproachTarget();
      direction += AvoidObstacles();
      task->speed = direction.Length();
      speed_actuator->SetLinearVelocity(direction);
    }

    void Round() {
      tick++;
      switch(state) {
        case State::START: Start(); break;
        case State::AT_GROUND: AtGround(); break;
        case State::TAKING_OFF: TakingOff(); break;
        case State::TAKEN_OFF: TakenOff(); break;
        case State::ARRIVED: Arrived(); break;
      }
    }
  
    void Reset() {
      state = START;
      waited_rounds = 0;
    }

    void ComputeHeadingToTarget() {
      argos::CVector3 position = positioning_sensor->GetReading().Position;
      delta_position = (position - last_position).Length();
      last_position = position;
      argos::CVector3& target_position = prez::GetTargetList()->at(task->target).position;
      argos::CVector3 direction = target_position - position;
      task->target_direction = direction;
      task->distance_from_target = direction.Length();
    }
  };
}
#endif//TASK_EXECUTOR_DEFAULT_HH
