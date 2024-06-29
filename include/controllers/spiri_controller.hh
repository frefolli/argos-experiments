#ifndef PREZ_SPIRI_CONTROLLER_HH
#define PREZ_SPIRI_CONTROLLER_HH
/** @file */
#include <support/task_allocators/default.hh>
#include <support/task_allocators/NN.hh>
#include <support/task_executors/default.hh>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <cmath>
#include <argos3/core/utility/math/rng.h>
#include <fstream>

namespace prez {
  /** Spiri Controller */
  class SpiriController : public argos::CCI_Controller {
    public:
     SpiriController();
     virtual ~SpiriController() {}
     
     virtual void Init(argos::TConfigurationNode& t_node);
     virtual void ControlStep();
     virtual void Reset();
     virtual void Destroy();
     
    private:
      /** Signals it's state and range to other spiris */
      argos::CCI_RangeAndBearingActuator* range_and_bearing_actuator;

      /** Logger */
      std::ofstream logfile;

      /** Composition over Inheritance */
      Task task;
      task_allocators::NN task_allocator;
      task_executors::Default task_executor;
  };
}
#endif
