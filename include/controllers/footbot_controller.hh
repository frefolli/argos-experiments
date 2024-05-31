#ifndef PREZ_FOOTBOT_HH
#define PREZ_FOOTBOT_HH
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/core/utility/math/rng.h>

namespace prez {
  class FootBotController : public argos::CCI_Controller {
    public:
     FootBotController();
     virtual ~FootBotController() {}

     /*
      * This function initializes the controller.
      * The 't_node' variable points to the <parameters> section in the XML
      * file in the <controllers><footbot_foraging_controller> section.
      */
     virtual void Init(argos::TConfigurationNode& t_node);

     /*
      * This function is called once every time step.
      * The length of the time step is set in the XML file.
      */
     virtual void ControlStep();

     /*
      * This function resets the controller to its state right after the
      * Init().
      * It is called when you press the reset button in the GUI.
      */
     virtual void Reset();

     /*
      * Called to cleanup what done by Init() when the experiment finishes.
      * In this example controller there is no need for clean anything up,
      * so the function could have been omitted. It's here just for
      * completeness.
      */
     virtual void Destroy() {}
    private:
  };
}
#endif
