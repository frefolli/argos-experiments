#include <argos3/core/utility/math/vector3.h>
#include <controllers/eyebot_controller.hh>
#include <argos3/core/utility/configuration/argos_configuration.h>

prez::EyeBotController::EyeBotController() :
  position_actuator(nullptr),
  speed_actuator(nullptr),
  range_and_bearing_actuator(nullptr),
  max_speed(1.0f) {}

void prez::EyeBotController::Init(argos::TConfigurationNode& t_node) {
  this->position_actuator = GetActuator<argos::CCI_QuadRotorPositionActuator>("position-actuator");
  this->speed_actuator = GetActuator<argos::CCI_QuadRotorSpeedActuator>("speed-actuator");
  this->range_and_bearing_actuator = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing-actuator");
  argos::GetNodeAttribute(t_node, "max_speed", max_speed);
}

void prez::EyeBotController::ControlStep() {
  argos::CVector3 pos = {0.0f, 0.0f, 3.0f};
  this->position_actuator->SetAbsolutePosition(pos);
}

void prez::EyeBotController::Reset() {
}

using namespace prez;
REGISTER_CONTROLLER(EyeBotController, "eyebot_controller")
