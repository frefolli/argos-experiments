#ifndef PREZ_FOOTBOT_HH
#define PREZ_FOOTBOT_HH
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_speed_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <cmath>

namespace prez {
  class EyeBotController : public argos::CCI_Controller {
    public:
     EyeBotController();
     virtual ~EyeBotController() {}

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
      argos::CCI_QuadRotorPositionActuator* position_actuator;
      argos::CCI_QuadRotorSpeedActuator* speed_actuator;
      argos::CCI_RangeAndBearingActuator* range_and_bearing_actuator;
      double_t max_speed;
  };
}
#endif
