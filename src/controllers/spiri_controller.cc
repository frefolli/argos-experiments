#include <argos3/core/utility/configuration/tinyxml/ticpp.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/vector3.h>
#include <cmath>
#include <controllers/spiri_controller.hh>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <string>
#include <support/target.hh>
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
  random_number_generator(nullptr),
  ID(-1)
  {}

void prez::SpiriController::Init(argos::TConfigurationNode& t_node) {
  position_actuator = GetActuator<argos::CCI_QuadRotorPositionActuator>("quadrotor_position");
  range_and_bearing_actuator = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");
  range_and_bearing_sensor = GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing");
  positioning_sensor = GetSensor<argos::CCI_PositioningSensor>("positioning");

  random_number_generator = argos::CRandom::CreateRNG("argos");
  Reset();
}
      
void prez::SpiriController::DoStart() {
  std::vector<prez::Target>* targets = prez::GetTargetList();
  if (targets->empty()) {
    std::cerr << "Warning: targets is empty" << std::endl;
  } else {
    argos::CRange<uint32_t> squadron_range(0, targets->size());
    squadron = random_number_generator->Uniform(squadron_range) % targets->size();

    mean_distance_from_targets = 0.0f;
    max_distance_from_targets = 0.0f;
    argos::CVector3 position = positioning_sensor->GetReading().Position;
    for (uint32_t index = 0; index < targets->size(); ++index) {
      distances_from_targets[index] = prez::Distance(position, targets->at(index).position);
      mean_distance_from_targets += distances_from_targets[index];
      max_distance_from_targets = std::max(max_distance_from_targets, distances_from_targets[index]);
    }
    mean_distance_from_targets /= targets->size();
    voting_session = 0;

    state = State::VOTING;
  }
}

void prez::SpiriController::DoVoting() {
  std::vector<prez::Target>* targets = prez::GetTargetList();
  std::unordered_map<uint32_t, uint32_t> formations;
  const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
  // std::cout << ID << " senses " << neighbours.size() << " neighbours" << std::endl;
  for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
    ++formations[neighbour.Data[KEY_SQUADRON]];
  }
  ++formations[squadron];

  if (formations[squadron] >= targets->at(squadron).force) {
    for (auto target : formations) {
      // std::cout << ID << " reads that " << target.first << " is #" << target.second << std::endl;
      if (target.first != squadron) {
        if (target.second < targets->at(target.first).force) {
          double_t reassignment_probability = 1 - (distances_from_targets[target.first] / max_distance_from_targets);
          bool reassign = random_number_generator->Bernoulli(reassignment_probability);
          if (reassign) {
            squadron = target.first;
            break;
          }
        } else if (distances_from_targets[squadron] > mean_distance_from_targets
                && distances_from_targets[target.first] < mean_distance_from_targets) {
          double_t reassignment_probability = 1 - ((double_t)targets->at(squadron).force / formations[squadron]);
          bool reassign = random_number_generator->Bernoulli(reassignment_probability);
          if (reassign) {
            squadron = target.first;
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
    argos::CVector3 target_position = current_position;
    target_position.SetZ(3.0f);
    position_actuator->SetAbsolutePosition(target_position);
    
    // Set state as TAKING_OFF
    state = State::TAKING_OFF;
  }
}

void prez::SpiriController::DoTakingOff() {
  // If i left 2.0f meters above ground i can consider myself taken off
  // I set and maintain current heading target
  argos::CVector3 current_position = positioning_sensor->GetReading().Position;
  if (current_position.GetZ() >= 2.0f) {
    state = State::TAKEN_OFF;
    position_actuator->SetAbsolutePosition(prez::GetTargetList()->at(squadron).position);
  }
}

void prez::SpiriController::DoTakenOff() {
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