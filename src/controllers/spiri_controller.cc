#include <argos3/core/utility/configuration/tinyxml/ticpp.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/vector3.h>
#include <cmath>
#include <controllers/spiri_controller.hh>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <string>
#include <support/squadrons.hh>
#include <support/vectors.hh>
#define KEY_STATE 0
#define KEY_ID 1
#define KEY_SQUADRON 2
#define MAX_VOTING_SESSIONS 100

prez::SpiriController::SpiriController() :
  position_actuator(nullptr),
  range_and_bearing_actuator(nullptr),
  range_and_bearing_sensor(nullptr),
  positioning_sensor(nullptr),
  proximity_sensor(nullptr),
  random_number_generator(nullptr),
  ID(-1)
  {}

void prez::SpiriController::Init(argos::TConfigurationNode& /* t_node */) {
  position_actuator = GetActuator<argos::CCI_QuadRotorPositionActuator>("quadrotor_position");
  range_and_bearing_actuator = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");
  range_and_bearing_sensor = GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing");
  positioning_sensor = GetSensor<argos::CCI_PositioningSensor>("positioning");
  proximity_sensor = GetSensor<argos::CCI_ProximitySensor>("proximity");

  random_number_generator = argos::CRandom::CreateRNG("argos");
  Reset();
}
      
void prez::SpiriController::DoStart() {
  std::vector<prez::Squadron>* squadrons = prez::GetSquadronList();
  if (squadrons->empty()) {
    std::cerr << "Warning: squadrons is empty" << std::endl;
  } else {
    argos::CRange<uint32_t> squadron_range(0, squadrons->size());
    squadron = random_number_generator->Uniform(squadron_range) % squadrons->size();

    mean_distance_from_squadrons = 0.0f;
    max_distance_from_squadrons = 0.0f;
    argos::CVector3 position = positioning_sensor->GetReading().Position;
    for (uint32_t index = 0; index < squadrons->size(); ++index) {
      distances_from_squadrons[index] = prez::Distance(position, squadrons->at(index).position);
      mean_distance_from_squadrons += distances_from_squadrons[index];
      max_distance_from_squadrons = std::max(max_distance_from_squadrons, distances_from_squadrons[index]);
    }
    mean_distance_from_squadrons /= squadrons->size();
    voting_session = 0;

    state = State::VOTING;
  }
}

void prez::SpiriController::DoVoting() {
  std::vector<prez::Squadron>* squadrons = prez::GetSquadronList();
  std::unordered_map<uint32_t, uint32_t> formations;
  const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
  // std::cout << ID << " senses " << neighbours.size() << " neighbours" << std::endl;
  for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
    ++formations[neighbour.Data[KEY_SQUADRON]];
  }
  ++formations[squadron];

  if (formations[squadron] >= squadrons->at(squadron).force) {
    for (auto formation : formations) {
      // std::cout << ID << " reads that " << squadron.first << " is #" << squadron.second << std::endl;
      if (formation.first != squadron) {
        if (formation.second < squadrons->at(formation.first).force) {
          double_t reassignment_probability = 1 - (distances_from_squadrons[formation.first] / max_distance_from_squadrons);
          bool reassign = random_number_generator->Bernoulli(reassignment_probability);
          if (reassign) {
            squadron = formation.first;
            break;
          }
        } else if (distances_from_squadrons[squadron] > mean_distance_from_squadrons
                && distances_from_squadrons[formation.first] < mean_distance_from_squadrons) {
          double_t reassignment_probability = 1 - ((double_t)squadrons->at(squadron).force / formations[squadron]);
          bool reassign = random_number_generator->Bernoulli(reassignment_probability);
          if (reassign) {
            squadron = formation.first;
            break;
          }
        }
      }
    }
  }

  if (voting_session > MAX_VOTING_SESSIONS) {
    state = State::AT_GROUND;
    std::cout << ID << " i want to go to " << squadron << std::endl;
  } else {
    ++voting_session;
  }
}

void prez::SpiriController::DoAtGround() {
  uint32_t waiting_queue = 0;
  const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
  for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
    if (neighbour.Data[KEY_SQUADRON] == squadron && neighbour.Data[KEY_ID] > ID && neighbour.Data[KEY_STATE] != State::TAKEN_OFF) {
      waiting_queue++;
    }
  }

  if (waiting_queue == 0) {
    argos::CVector3 current_position = positioning_sensor->GetReading().Position;
    argos::CVector3 squadron_position = current_position;
    squadron_position.SetZ(3.0f);
    position_actuator->SetAbsolutePosition(squadron_position);
    
    // Set state as TAKING_OFF
    state = State::TAKING_OFF;
  }
}

void prez::SpiriController::DoTakingOff() {
  // If i left 2.0f meters above ground i can consider myself taken off
  // I set and maintain current heading squadron
  argos::CVector3 current_position = positioning_sensor->GetReading().Position;
  if (current_position.GetZ() >= 2.0f) {
    state = State::TAKEN_OFF;
  }
}

void prez::SpiriController::DoTakenOff() {
  position_actuator->SetAbsolutePosition(prez::GetSquadronList()->at(squadron).position);
  /*
  argos::CVector3 direction(0.0f, 0.0f, 0.0f);
  direction += ApproachSquadron();
  // direction += AvoidObstacles();
  position_actuator->SetRelativePosition(argos::CVector3(direction.GetX(), direction.GetY(), 0.0f));
  */
}

argos::CVector3 prez::SpiriController::ApproachSquadron() {
  argos::CVector3 current_position = positioning_sensor->GetReading().Position;
  argos::CVector3 direction = prez::GetSquadronList()->at(squadron).position - current_position;
  direction = direction.Normalize();
  return direction * 0.1f;
}

argos::CVector3 prez::SpiriController::AvoidObstacles() {
  argos::CVector3 direction(0.0f, 0.0f, 0.0f);
  return direction;
}

void prez::SpiriController::ControlStep() {
  switch (state) {
    case State::START: {
      DoStart();
    }; break;
    case State::VOTING: {
      DoVoting();
    }; break;
    case State::AT_GROUND: {
      DoAtGround();
    }; break;
    case State::TAKING_OFF: {
      DoTakingOff();
    }; break;
    case State::TAKEN_OFF: {
      DoTakenOff();
    }; break;
  }
  
  range_and_bearing_actuator->SetData(KEY_STATE, state);
  range_and_bearing_actuator->SetData(KEY_ID, ID);
  range_and_bearing_actuator->SetData(KEY_SQUADRON, squadron);
}

void prez::SpiriController::Reset() {
  ID = std::stoi(GetId().substr(2));
  state = State::START;
  squadron = -1;
}

using namespace prez;
REGISTER_CONTROLLER(SpiriController, "spiri_controller")
