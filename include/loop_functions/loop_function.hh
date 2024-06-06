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
      virtual void Reset();

    private:
      void InitializeSquadrons();
    
      argos::CRandom::CRNG* random_number_generator;
      
      /** Configuration for Squadron Initialization */
      struct Squadrons {
        uint32_t number_of_squadrons = 4;
        uint32_t minimum_force = 5;
        uint32_t maximum_force = 7;
        argos::CVector3 minimum_position;
        argos::CVector3 maximum_position;
      } squadrons_config;
      void ConfigureSquadrons(argos::TConfigurationNode& config);
  };
}
#endif
