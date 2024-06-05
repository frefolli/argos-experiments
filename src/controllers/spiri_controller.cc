#include <argos3/core/utility/math/vector3.h>
#include <controllers/spiri_controller.hh>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <iostream>

prez::SpiriController::SpiriController() :
  position_actuator(nullptr),
  range_and_bearing_actuator(nullptr),
  range_and_bearing_sensor(nullptr),
  positioning_sensor(nullptr),
  random_number_generator(nullptr)
  {}

void prez::SpiriController::Init(argos::TConfigurationNode& t_node) {
  position_actuator = GetActuator<argos::CCI_QuadRotorPositionActuator>("quadrotor_position");
  range_and_bearing_actuator = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");
  range_and_bearing_sensor = GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing");
  positioning_sensor = GetSensor<argos::CCI_PositioningSensor>("positioning");

  random_number_generator = argos::CRandom::CreateRNG("argos");
  Reset();
}

void prez::SpiriController::ControlStep() {
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
        }
      }
    }; break;
    case State::TAKING_OFF: {
      // If i left 2.0f meters above ground i can consider myself taken off
      // I maintain current heading target
      argos::CVector3 current_position = positioning_sensor->GetReading().Position;
      if (current_position.GetZ() >= 2.0f) {
        state = State::TAKEN_OFF;
      }
    }; break;
    case State::TAKEN_OFF: {
      // Idle
    }; break;
  }
  range_and_bearing_actuator->SetData(0, state);
}

void prez::SpiriController::Reset() {
  state = State::AT_GROUND;
}

using namespace prez;
REGISTER_CONTROLLER(SpiriController, "spiri_controller")
