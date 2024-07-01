#include <loop_functions/loop_function.hh>
#include <support/squadrons.hh>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <support/vectors.hh>
#include <argos3/plugins/simulator/entities/light_entity.h>

prez::LoopFunction::LoopFunction() :
  random_number_generator(NULL) {
}

void prez::LoopFunction::Init(argos::TConfigurationNode& t_node) {
  ConfigureSquadrons(argos::GetNode(t_node, "squadrons"));
  random_number_generator = argos::CRandom::CreateRNG("argos");
  Reset();
}

void prez::LoopFunction::ConfigureSquadrons(argos::TConfigurationNode& config) {
  argos::GetNodeAttribute(config, "minimum_position", squadrons_config.minimum_position);
  argos::GetNodeAttribute(config, "maximum_position", squadrons_config.maximum_position);
  argos::GetNodeAttribute(config, "minimum_force", squadrons_config.minimum_force);
  argos::GetNodeAttribute(config, "maximum_force", squadrons_config.maximum_force);
  argos::GetNodeAttribute(config, "number_of_squadrons", squadrons_config.number_of_squadrons);
}

void prez::LoopFunction::Reset() {
  InitializeSquadrons();
}

void prez::LoopFunction::InitializeSquadrons() {
  argos::CRange<uint32_t> force_distribution({squadrons_config.minimum_force,squadrons_config.maximum_force});
  argos::CRange<double_t> position_X_distribution({squadrons_config.minimum_position.GetX(),squadrons_config.maximum_position.GetX()});
  argos::CRange<double_t> position_Y_distribution({squadrons_config.minimum_position.GetY(),squadrons_config.maximum_position.GetY()});
  argos::CRange<double_t> position_Z_distribution({squadrons_config.minimum_position.GetZ(),squadrons_config.maximum_position.GetZ()});
  std::vector<prez::Squadron>* squadrons = prez::GetSquadronList();
  squadrons->clear();
  prez::Squadron new_squadron;
  for (uint32_t i = 0; i < squadrons_config.number_of_squadrons; ++i) {
    new_squadron.position.SetX(random_number_generator->Uniform(position_X_distribution));
    new_squadron.position.SetY(random_number_generator->Uniform(position_Y_distribution));
    new_squadron.position.SetZ(random_number_generator->Uniform(position_Z_distribution));
    new_squadron.force = random_number_generator->Uniform(force_distribution);
    std::cout << "new squadron: " << new_squadron << std::endl;
    squadrons->push_back(new_squadron);

    argos::CLightEntity* le = new argos::CLightEntity(
      std::to_string(i),
      argos::CVector3(new_squadron.position.GetX(), new_squadron.position.GetY(), new_squadron.position.GetY()),
      argos::CColor::YELLOW,
      1
      );
    AddEntity(*le);
  }
}

using namespace prez;
REGISTER_LOOP_FUNCTIONS(LoopFunction, "loop_function")
