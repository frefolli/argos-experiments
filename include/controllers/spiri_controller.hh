#ifndef PREZ_SPIRI_CONTROLLER_HH
#define PREZ_SPIRI_CONTROLLER_HH
/** @file */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <cmath>
#include <argos3/core/utility/math/rng.h>
#include <unordered_map>

namespace prez {
  /** Spiri Controller */
  class SpiriController : public argos::CCI_Controller {
    public:
     SpiriController();
     virtual ~SpiriController() {}

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
      /** Move the spiri to a target position
       * For some reason you can have either a position_actuator or a speed_actuator, not both.
      */
      argos::CCI_QuadRotorPositionActuator* position_actuator;
      
      /** Signals it's state and range to other spiris */
      argos::CCI_RangeAndBearingActuator* range_and_bearing_actuator;
      
      /** Recieves state and range of other robots */
      argos::CCI_RangeAndBearingSensor* range_and_bearing_sensor;
      
      /** Read current spiri position */
      argos::CCI_PositioningSensor* positioning_sensor;
      
      /**  Random Number Generator */
      argos::CRandom::CRNG* random_number_generator;

      /** State enum */
      enum State {
        VOTING, // Need to vote a squadron
        AT_GROUND,  // Ready to takeoff
        TAKING_OFF, // Reaching Battle Stations
        TAKEN_OFF   // In Formation
      } state;

      /** My ID */
      uint32_t ID;

      /** My Squadron */
      uint32_t squadron;
      std::unordered_map<uint32_t, double_t> distances_from_targets;
      double_t max_distance_from_targets;
      double_t mean_distance_from_targets;
      uint32_t voting_session;
  };
}
#endif
