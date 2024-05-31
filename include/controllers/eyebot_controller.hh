#ifndef PREZ_FOOTBOT_HH
#define PREZ_FOOTBOT_HH
/** @file */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
// #include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_speed_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
// #include <argos3/plugins/robots/eye-bot/control_interface/ci_eyebot_light_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <cmath>
#include <random>
#include <argos3/core/utility/math/rng.h>

namespace prez {
  /** EyeBot Controller */
  class EyeBotController : public argos::CCI_Controller {
    public:
     EyeBotController();
     virtual ~EyeBotController() {}

     /*
      * This function initializes the controller.
      * The 't_node' variable points to the <parameters> section in the XML file in the <controllers><footbot_foraging_controller> section.
      */
     virtual void Init(argos::TConfigurationNode& t_node);

     /*
      * This function is called once every time step.
      * The length of the time step is set in the XML file.
      */
     virtual void ControlStep();

     /*
      * This function resets the controller to its state right after the Init().
      * It is called when you press the reset button in the GUI.
      */
     virtual void Reset();

     /*
      * Called to cleanup what done by Init() when the experiment finishes.
      * In this example controller there is no need for clean anything up, so the function could have been omitted.
      * It's here just for completeness.
      */
     virtual void Destroy() {}
    private:
      /** Move the eyebot to a target position
       * For some reason you can have either a position_actuator or a speed_actuator, not both.
      */
      argos::CCI_QuadRotorPositionActuator* position_actuator;
      
      /** Move the eyebot with decided speed vector */
      // argos::CCI_QuadRotorSpeedActuator* speed_actuator;
      
      /** Signals it's state and range to other eyebots */
      argos::CCI_RangeAndBearingActuator* range_and_bearing_actuator;
      
      /** Recieves state and range of other robots */
      argos::CCI_RangeAndBearingSensor* range_and_bearing_sensor;
      
      /** Returns the position of the closest light object */
      // argos::CCI_EyeBotLightSensor* eyebot_light_sensor;
      
      /** Read current eyebot position */
      argos::CCI_PositioningSensor* positioning_sensor;
      
      /**  Random Number Generator */
      argos::CRandom::CRNG* random_number_generator;

      /** State enum */
      enum State {
        AT_GROUND = 0, TAKING_OFF, TAKEN_OFF
      } state;
  };
}
#endif