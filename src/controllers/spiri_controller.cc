#include <argos3/core/utility/configuration/tinyxml/ticpp.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/vector3.h>
#include <cmath>
#include <controllers/spiri_controller.hh>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <string>
#include <support/squadrons.hh>
#include <support/vectors.hh>

constexpr uint32_t KEY_STATE = 0;
constexpr uint32_t KEY_ID = 1;
constexpr uint32_t KEY_SQUADRON = 2;
constexpr uint32_t MAX_VOTING_SESSIONS = 20;
constexpr double_t MAX_COLLISION_AVOIDANCE_DISTANCE = 1000.0f;;
#define USE_CAD_GLJ

prez::SpiriController::SpiriController() :
  position_actuator(nullptr),
  range_and_bearing_actuator(nullptr),
  range_and_bearing_sensor(nullptr),
  positioning_sensor(nullptr),
  random_number_generator(nullptr),
  ID(-1)
  {}

void prez::SpiriController::Init(argos::TConfigurationNode& /* t_node */) {
  position_actuator = GetActuator<argos::CCI_QuadRotorPositionActuator>("quadrotor_position");
  range_and_bearing_actuator = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");
  range_and_bearing_sensor = GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing");
  positioning_sensor = GetSensor<argos::CCI_PositioningSensor>("positioning");

  logfile.open(GetId() + ".log");
  random_number_generator = argos::CRandom::CreateRNG("argos");
  Reset();
}

void prez::SpiriController::Destroy() {
  logfile.close();
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
      distances_from_squadrons[index] = (position - squadrons->at(index).position).Length();
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
  logfile << ID << " senses " << neighbours.size() << " neighbours" << std::endl;
  for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
    ++formations[neighbour.Data[KEY_SQUADRON]];
  }
  ++formations[squadron];

  if (formations[squadron] >= squadrons->at(squadron).force) {
    for (auto formation : formations) {
      logfile << ID << " reads that " << formation.first << " is #" << formation.second << std::endl;
      if (formation.first != squadron) {
        if (formation.second < squadrons->at(formation.first).force) {
          double_t reassignment_probability = 1 - (distances_from_squadrons[formation.first] / max_distance_from_squadrons);
          bool reassign = random_number_generator->Bernoulli(reassignment_probability);
          if (reassign) {
            squadron = formation.first;
            break;
          }
        }/* else if (distances_from_squadrons[squadron] > mean_distance_from_squadrons
                && distances_from_squadrons[formation.first] < mean_distance_from_squadrons) {
          double_t reassignment_probability = 1 - ((double_t)squadrons->at(squadron).force / formations[squadron]);
          bool reassign = random_number_generator->Bernoulli(reassignment_probability);
          if (reassign) {
            squadron = formation.first;
            break;
          }
        }*/
      }
    }
  }

  if (voting_session > MAX_VOTING_SESSIONS) {
    state = State::AT_GROUND;
    logfile << ID << " i want to go to " << squadron << std::endl;
  } else {
    ++voting_session;
  }
}

void prez::SpiriController::DoAtGround() {
  uint32_t waiting_queue = 0;
  const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
  for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
    if (neighbour.Range < MAX_COLLISION_AVOIDANCE_DISTANCE) {
      if (neighbour.Data[KEY_ID] > ID && neighbour.Data[KEY_STATE] != State::TAKEN_OFF) {
        waiting_queue++;
      }
    }
  }

  if (waiting_queue == 0) {
    argos::CVector3 current_position = positioning_sensor->GetReading().Position;
    argos::CVector3 squadron_position = current_position;
    squadron_position.SetZ(3.0f);
    position_actuator->SetAbsolutePosition(squadron_position);
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
  argos::CVector3 direction(0.0f, 0.0f, 0.0f);
  direction += ApproachSquadron();
  direction += AvoidObstacles();

  position_actuator->SetRelativePosition(direction);
}

argos::CVector3 prez::SpiriController::ApproachSquadron() {
  argos::CVector3 position = positioning_sensor->GetReading().Position;
  argos::CQuaternion orientation = positioning_sensor->GetReading().Orientation;
  argos::CVector3& target_position = prez::GetSquadronList()->at(squadron).position;
  argos::CVector3 direction = target_position - position;
  double_t Z = direction.GetZ();
  direction = direction.Rotate(orientation.Inverse());
  direction.SetZ(Z);
  direction = direction.Normalize();
  return direction;
}

argos::Real TargetDistance = 300.0f;
argos::Real Gain = 25.0f;
argos::Real Exponent = 1.5f;
argos::Real MaxInteraction = 2.0f;
argos::Real GeneralizedLennardJones(argos::Real f_distance) {
   argos::Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

argos::Real GravitationalPotential(argos::Real f_distance) {
  return ::pow(::abs(TargetDistance - f_distance) / f_distance, 2.0f);
}

argos::CVector3 prez::SpiriController::AvoidObstacles() {
  argos::CVector2 direction(0.0f, 0.0f);
  const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
  if (neighbours.size() > 0) {
    for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
      if ((neighbour.Data[KEY_STATE] == State::TAKING_OFF
        || neighbour.Data[KEY_STATE] == State::TAKEN_OFF)
        && neighbour.Range < MAX_COLLISION_AVOIDANCE_DISTANCE) {
        argos::CVector2 avoidance(
          #ifdef USE_CAD_GLJ
          GeneralizedLennardJones
          #else
          GravitationalPotential
          #endif
          (neighbour.Range), neighbour.HorizontalBearing);
        direction += avoidance;
      }
    }
    direction = direction / neighbours.size();
  }
  if (direction.Length() > MaxInteraction) {
    direction = direction.Normalize() * MaxInteraction;
  }
  return argos::CVector3(direction.GetX(), direction.GetY(), 0.0f);
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
