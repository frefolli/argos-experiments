#include <argos3/core/utility/math/vector3.h>
#include <controllers/spiri_controller.hh>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <iostream>
#define KEY_STATE 0
#define KEY_ID 1

prez::SpiriController::SpiriController() :
  position_actuator(nullptr),
  range_and_bearing_actuator(nullptr),
  range_and_bearing_sensor(nullptr),
  positioning_sensor(nullptr),
  random_number_generator(nullptr),
  ID(-1)
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
      uint32_t waiting_queue = 0;
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
        if (neighbour.Data[KEY_ID] > ID && neighbour.Data[KEY_STATE] != State::TAKEN_OFF) {
          waiting_queue++;
        }
      }

      if (waiting_queue == 0) {
        argos::CVector3 current_position = positioning_sensor->GetReading().Position;
        argos::CVector3 target_position = current_position;
        target_position.SetZ(3.0f);
        position_actuator->SetAbsolutePosition(target_position);
        
        // Set state as TAKING_OFF
        state = State::TAKING_OFF;
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
  range_and_bearing_actuator->SetData(KEY_STATE, state);
  range_and_bearing_actuator->SetData(KEY_ID, ID);
}

void prez::SpiriController::Reset() {
  state = State::AT_GROUND;
  ID = std::stoi(GetId().substr(2));
  range_and_bearing_actuator->SetData(KEY_STATE, state);
  range_and_bearing_actuator->SetData(KEY_ID, ID);
}

using namespace prez;
REGISTER_CONTROLLER(SpiriController, "spiri_controller")
