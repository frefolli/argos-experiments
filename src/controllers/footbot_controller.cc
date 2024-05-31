#include <controllers/footbot_controller.hh>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>

prez::FootBotController::FootBotController() {}

void prez::FootBotController::Init(argos::TConfigurationNode& t_node) {
}

void prez::FootBotController::ControlStep() {
}

void prez::FootBotController::Reset() {
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
using namespace prez;
REGISTER_CONTROLLER(FootBotController, "footbot_controller")
