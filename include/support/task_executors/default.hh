#ifndef TASK_EXECUTOR_DEFAULT_HH
#define TASK_EXECUTOR_DEFAULT_HH
/** @file default.hh */
#include <argos3/core/utility/math/angles.h>
#include <support/task_executor.hh>
#include <support/rab.hh>
#include <support/squadrons.hh>
#include <support/task.hh>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_speed_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <cstdint>
namespace prez::task_executors {
  class Default : public TaskExecutor {
    static constexpr uint32_t MAX_WAITED_ROUNDS = 20;
    static constexpr double_t MAX_COLLISION_AVOIDANCE_DISTANCE = 500.0f;

    static constexpr argos::Real TARGETDISTANCE = 300.0f;
    static constexpr argos::Real MAXINTERACTION = 10.0f;
    #define USE_CAD_GLJ

    public:
    enum State {
      START, AT_GROUND, TAKING_OFF, TAKEN_OFF, IDLE
    } state = START;
    uint32_t waited_rounds = 0;

    Task* task;
    argos::CCI_PositioningSensor* positioning_sensor;
    argos::CCI_RangeAndBearingSensor* range_and_bearing_sensor;
    argos::CCI_QuadRotorSpeedActuator* speed_actuator;

    void Start() {
      ++waited_rounds;
      if (waited_rounds > MAX_WAITED_ROUNDS) {
        state = State::AT_GROUND;
      } else {
        ++waited_rounds;
      }
    }

    void AtGround() {
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
      // If i left 2.0f meters above ground i can consider myself taken off
      // I set and maintain current heading squadron
      argos::CVector3 current_position = positioning_sensor->GetReading().Position;
      if (current_position.GetZ() >= 1.5f) {
        speed_actuator->SetLinearVelocity({0.0f, 0.0f, 0.0f});
        state = State::TAKEN_OFF;
      }
    }

    argos::CVector3 ApproachSquadron() {
      argos::CVector3 position = positioning_sensor->GetReading().Position;
      argos::CQuaternion orientation = positioning_sensor->GetReading().Orientation;
      argos::CVector3& target_position = prez::GetSquadronList()->at(task->squadron).position;
      argos::CVector3 direction = target_position - position;
      double_t Z = direction.GetZ();
      direction = direction.Rotate(orientation.Inverse());
      direction.SetZ(Z);
      direction = direction.Normalize() * MAXINTERACTION;
      return direction;
    }

    argos::Real GravitationalPotential(argos::Real f_distance) {
      double_t tevere = TARGETDISTANCE - f_distance;
      if (tevere < 0) {
        return 0.0f;
      }
      return ::pow(::abs(tevere) / f_distance, 2.0f);
    }

    argos::CVector3 AvoidObstacles() {
      argos::CVector2 direction(0.0f, 0.0f);
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      if (neighbours.size() > 0) {
        for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
          if ((neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::TAKING_OFF
            || neighbour.Data[prez::RABKey::TASK_EXECUTOR_STATE] == State::TAKEN_OFF)
            && neighbour.Range < MAX_COLLISION_AVOIDANCE_DISTANCE) {
            argos::CVector2 avoidance(
              GravitationalPotential
              (neighbour.Range), neighbour.HorizontalBearing);
            direction += avoidance;
          }
        }
        direction = direction / neighbours.size();
      }
      if (direction.Length() > MAXINTERACTION) {
        direction = direction.Normalize() * MAXINTERACTION;
      }
      return argos::CVector3(direction.GetX(), direction.GetY(), 0.0f);
    }

    void TakenOff() {
      argos::CVector3 direction(0.0f, 0.0f, 0.0f);
      direction += ApproachSquadron();
      direction += AvoidObstacles();

      speed_actuator->SetLinearVelocity(direction);
    }

    void Idle() {}

    void Round() {
      switch(state) {
        case State::START: Start(); break;
        case State::AT_GROUND: AtGround(); break;
        case State::TAKING_OFF: TakingOff(); break;
        case State::TAKEN_OFF: TakenOff(); break;
        case State::IDLE: Idle(); break;
      }
    }
  
    void Reset() {
      state = START;
      waited_rounds = 0;
    }
  };
}
#endif//TASK_EXECUTOR_DEFAULT_HH
