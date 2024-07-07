#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <fstream>
#include <ostream>
#include <support/targets.hh>
#include <support/vectors.hh>
#include <support/logging.hh>
#include <loop_functions/loop_function.hh>

prez::LoopFunction::LoopFunction() : random_number_generator(NULL)
{
}

void prez::LoopFunction::Init(argos::TConfigurationNode &t_node)
{
  ConfigureTargets(argos::GetNode(t_node, "targets"));
  random_number_generator = argos::CRandom::CreateRNG("argos");
  Reset();
}

void prez::LoopFunction::ConfigureTargets(argos::TConfigurationNode &config)
{
  /*we valorize targets_config with the values read from the config file*/
  argos::GetNodeAttribute(config, "minimum_position", targets_config.minimum_position);
  argos::GetNodeAttribute(config, "maximum_position", targets_config.maximum_position);
  argos::GetNodeAttribute(config, "required_target_force", targets_config.required_target_force);
  argos::GetNodeAttribute(config, "number_of_targets", targets_config.number_of_targets);
}

void prez::LoopFunction::Reset()
{
  InitializeTargets();
}

void prez::LoopFunction::InitializeTargets()
{
  argos::CRange<double_t> position_X_distribution({targets_config.minimum_position.GetX(), targets_config.maximum_position.GetX()});
  argos::CRange<double_t> position_Y_distribution({targets_config.minimum_position.GetY(), targets_config.maximum_position.GetY()});
  argos::CRange<double_t> position_Z_distribution({targets_config.minimum_position.GetZ(), targets_config.maximum_position.GetZ()});
  std::vector<prez::Target> *targets = prez::GetTargetList();
  targets->clear();
  prez::Target new_target;

  std::ofstream logfile(prez::TargetsLogfile());
  logfile
      << "TargetID,PosX,PosY,PosZ,Force"
      << std::endl;
  for (uint32_t i = 0; i < targets_config.number_of_targets; ++i)
  {
    new_target.position.SetX(random_number_generator->Uniform(position_X_distribution));
    new_target.position.SetY(random_number_generator->Uniform(position_Y_distribution));
    new_target.position.SetZ(random_number_generator->Uniform(position_Z_distribution));
    new_target.force = targets_config.required_target_force;
    logfile
        << i << ","
        << new_target.position << ","
        << new_target.force
        << std::endl;
    targets->push_back(new_target);

    argos::CLightEntity *le = new argos::CLightEntity(
        std::to_string(i),
        argos::CVector3(new_target.position.GetX(), new_target.position.GetY(), new_target.position.GetZ()),
        argos::CColor::YELLOW,
        1);
    AddEntity(*le);
  }
  logfile.close();
}

using namespace prez;
REGISTER_LOOP_FUNCTIONS(LoopFunction, "loop_function")
