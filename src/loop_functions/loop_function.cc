#include <loop_functions/loop_function.hh>
#include <support/target.hh>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <support/vectors.hh>

prez::LoopFunction::LoopFunction() :
  random_number_generator(NULL) {
}

void prez::LoopFunction::Init(argos::TConfigurationNode& t_node) {
  ConfigureTargets(argos::GetNode(t_node, "targets"));
  random_number_generator = argos::CRandom::CreateRNG("argos");
  Reset();
}

void prez::LoopFunction::ConfigureTargets(argos::TConfigurationNode& config) {
  argos::GetNodeAttribute(config, "minimum_position", targets_config.minimum_position);
  argos::GetNodeAttribute(config, "maximum_position", targets_config.maximum_position);
  argos::GetNodeAttribute(config, "minimum_force", targets_config.minimum_force);
  argos::GetNodeAttribute(config, "maximum_force", targets_config.maximum_force);
  argos::GetNodeAttribute(config, "number_of_targets", targets_config.number_of_targets);
}

void prez::LoopFunction::Reset() {
  InitializeTargets();
}

void prez::LoopFunction::InitializeTargets() {
  argos::CRange<uint32_t> force_distribution({targets_config.minimum_force,targets_config.maximum_force});
  argos::CRange<double_t> position_X_distribution({targets_config.minimum_position.GetX(),targets_config.maximum_position.GetX()});
  argos::CRange<double_t> position_Y_distribution({targets_config.minimum_position.GetY(),targets_config.maximum_position.GetY()});
  argos::CRange<double_t> position_Z_distribution({targets_config.minimum_position.GetZ(),targets_config.maximum_position.GetZ()});
  std::vector<prez::Target>* targets = prez::GetTargetList();
  targets->clear();
  prez::Target new_target;
  for (uint32_t i = 0; i < targets_config.number_of_targets; ++i) {
    new_target.position.SetX(random_number_generator->Uniform(position_X_distribution));
    new_target.position.SetY(random_number_generator->Uniform(position_Y_distribution));
    new_target.position.SetZ(random_number_generator->Uniform(position_Z_distribution));
    new_target.force = random_number_generator->Uniform(force_distribution);
    targets->push_back(new_target);
  }
}

using namespace prez;
REGISTER_LOOP_FUNCTIONS(LoopFunction, "loop_function")