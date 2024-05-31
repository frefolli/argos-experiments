#include <argos3/core/utility/math/vector3.h>
#include <controllers/eyebot_controller.hh>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <iostream>

prez::EyeBotController::EyeBotController() :
  position_actuator(nullptr),
  // speed_actuator(nullptr),
  range_and_bearing_actuator(nullptr),
  range_and_bearing_sensor(nullptr),
  // eyebot_light_sensor(nullptr),
  positioning_sensor(nullptr),
  random_number_generator(nullptr)
  {}

void prez::EyeBotController::Init(argos::TConfigurationNode& t_node) {
  position_actuator = GetActuator<argos::CCI_QuadRotorPositionActuator>("quadrotor_position");
  // speed_actuator = GetActuator<argos::CCI_QuadRotorSpeedActuator>("quadrotor_speed");
  range_and_bearing_actuator = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");
  range_and_bearing_sensor = GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing");
  // eyebot_light_sensor = GetSensor<argos::CCI_EyeBotLightSensor>("eyebot_light");
  positioning_sensor = GetSensor<argos::CCI_PositioningSensor>("positioning");

  random_number_generator = argos::CRandom::CreateRNG("argos");
  Reset();
}

void prez::EyeBotController::ControlStep() {
  switch (state) {
    case State::AT_GROUND: {
      uint32_t count_taking_off = 0;
      uint32_t count_taken_off = 0;
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
        if (neighbour.Data[0] == State::TAKING_OFF) {
          count_taking_off++;
        } else if (neighbour.Data[0] == State::TAKEN_OFF) {
          count_taken_off++;
        }
      }

      if (count_taking_off == 0) {
        // Probabilistic backoff, P(take_off) = (count_taken_off+1) / (neighbours.size())
        if (random_number_generator->Bernoulli((double_t)(count_taken_off+1) / (neighbours.size()))) {
          // Set target position at above 3.0f meters
          argos::CVector3 current_position = positioning_sensor->GetReading().Position;
          argos::CVector3 target_position = current_position;
          target_position.SetZ(3.0f);
          position_actuator->SetAbsolutePosition(target_position);
          
          // Set state as TAKING_OFF
          state = State::TAKING_OFF;
          range_and_bearing_actuator->SetData(0, state);
        }
      }
    }; break;
    case State::TAKING_OFF: {
      // If i left 2.0f meters above ground i can consider myself taken off
      // I maintain current heading target
      argos::CVector3 current_position = positioning_sensor->GetReading().Position;
      if (current_position.GetZ() >= 2.0f) {
        state = State::TAKEN_OFF;
        range_and_bearing_actuator->SetData(0, state);
      }
    }; break;
    case State::TAKEN_OFF: {
      // Idle
      range_and_bearing_actuator->SetData(0, state);
    }; break;
  }
}

void prez::EyeBotController::Reset() {
  state = State::AT_GROUND;
}

using namespace prez;
REGISTER_CONTROLLER(EyeBotController, "eyebot_controller")