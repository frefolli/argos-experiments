#ifndef PREZ_LOOP_FUNCTIONS_LOOP_FUNCTION_HH
#define PREZ_LOOP_FUNCTIONS_LOOP_FUNCTION_HH
/** @file loop_function.hh */
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

namespace prez {
  class LoopFunction : public argos::CLoopFunctions {
    public:
      LoopFunction();
      virtual ~LoopFunction() {}
      virtual void Init(argos::TConfigurationNode& t_tree);
      /* This method restore the state of the simulation at it was right after Init() was called
      */
      virtual void Reset();

    private:
      void InitializeTargets();
    
      argos::CRandom::CRNG* random_number_generator;
      
      /** Configuration for Target Initialization */
      struct Targets {
        uint32_t number_of_targets = 4;
        uint32_t minimum_force = 5;
        uint32_t maximum_force = 7;
        argos::CVector3 minimum_position;
        argos::CVector3 maximum_position;
      } targets_config;
      void ConfigureTargets(argos::TConfigurationNode& config);
  };
}
#endif
