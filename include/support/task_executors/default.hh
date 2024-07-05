#ifndef TASK_EXECUTOR_DEFAULT_HH
#define TASK_EXECUTOR_DEFAULT_HH
/** @file default.hh */
#include "support/coordination.hh"
#include <argos3/core/utility/math/angles.h>
#include <support/task_executor.hh>
#include <support/rab.hh>
#include <support/targets.hh>
#include <support/task.hh>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_speed_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <cstdint>

namespace prez::task_executors {
  class Default : public TaskExecutor {
    static constexpr uint32_t MAX_WAITED_ROUNDS = 20;
    static constexpr double_t MAX_COLLISION_AVOIDANCE_DISTANCE = 500.0f;

    static constexpr argos::Real TARGETDISTANCE = 500.0f;
    static constexpr argos::Real MAXINTERACTION = 10.0f;
    bool singletonDone = false;
    #define USE_CAD_GLJ

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
        if (neighbour.Range < MAX_COLLISION_AVOIDANCE_DISTANCE) {
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
      }
    }

    argos::CVector3 ApproachTarget() {
      ComputeHeadingToTarget();
      if (task->target_direction.Length() > MAXINTERACTION) {
        argos::CVector3 direction = task->target_direction.Normalize() * MAXINTERACTION;
        return direction;
      }
      return task->target_direction;
    }

    argos::Real GravitationalPotential(argos::Real f_distance) {
      double_t tevere = TARGETDISTANCE - f_distance;
      if (tevere < 0) {
        return 0.0f;
      }
      return ::abs(tevere) / f_distance;
    }

    argos::CVector3 AvoidObstacles() {
      argos::CVector2 direction(0.0f, 0.0f);
      argos::CQuaternion orientation = positioning_sensor->GetReading().Orientation;
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      uint32_t n_of_forces = 0;
      for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
        if ((neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::TAKING_OFF
          || neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::TAKEN_OFF)
          && neighbour.Range < MAX_COLLISION_AVOIDANCE_DISTANCE) {
          argos::CVector2 avoidance(
            GravitationalPotential
            (neighbour.Range), neighbour.HorizontalBearing);
          n_of_forces++;
          direction -= avoidance;
        }
      }
      if (n_of_forces > 0) {
        direction = direction / n_of_forces;
        argos::CVector3 avoidance = argos::CVector3(direction.GetX(), direction.GetY(), 0.0f);
        avoidance = avoidance.Rotate(orientation);
        avoidance.SetZ(0.0f);
        if (avoidance.Length() > MAXINTERACTION) {
          avoidance = avoidance.Normalize() * MAXINTERACTION;
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

      if(task->speed < 0.15f) {
        prez::Coordination::GetInstance().Finished();
        state = State::ARRIVED;
        argos::CVector3 stop(0.0f, 0.0f, 0.0f);
        speed_actuator->SetLinearVelocity(stop);
      }
    }

    void Arrived() {
      if(!singletonDone){
        singletonDone = true;     
      }
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
      singletonDone=false;
    }

    void ComputeHeadingToTarget() {
      argos::CVector3 position = positioning_sensor->GetReading().Position;
      argos::CVector3& target_position = prez::GetTargetList()->at(task->target).position;
      argos::CVector3 direction = target_position - position;
      task->target_direction = direction;
      task->distance_from_target = direction.Length();
    }
  };
}
#endif//TASK_EXECUTOR_DEFAULT_HH
