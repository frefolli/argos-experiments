#ifndef TASK_ALLOCATORS_DEFAULT_HH
#define TASK_ALLOCATORS_DEFAULT_HH
/** @file default.hh */
#include <support/task_allocator.hh>
#include <support/targets.hh>
#include <support/rab.hh>
#include <support/task.hh>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/rng.h>

namespace prez::task_allocators {
  class Default : public TaskAllocator {
    static constexpr uint32_t MAX_VOTING_SESSIONS = 20;

    public:
    enum State {
      START, VOTING, IDLE
    } state = START;
    uint32_t voting_sessions = 0;

    std::unordered_map<uint32_t, double_t> distances_from_targets;
    double_t max_distance_from_targets;
    double_t mean_distance_from_targets;

    Task* task;
    argos::CRandom::CRNG* random_number_generator;
    argos::CCI_PositioningSensor* positioning_sensor;
    argos::CCI_RangeAndBearingSensor* range_and_bearing_sensor;

    void Start() {
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      uint32_t const n = argos::CSimulator::GetInstance().GetSpace().GetEntitiesByType("eye-bot").size();
      assert(n == neighbours.size() + 1);

      std::vector<prez::Target>* targets = prez::GetTargetList();
      if (targets->empty()) {
        std::cerr << "Warning: targets is empty" << std::endl;
      } else {
        argos::CRange<uint32_t> target_range(0, targets->size());
        task->target = random_number_generator->Uniform(target_range) % targets->size();

        mean_distance_from_targets = 0.0f;
        max_distance_from_targets = 0.0f;
        argos::CVector3 position = positioning_sensor->GetReading().Position;
        for (uint32_t index = 0; index < targets->size(); ++index) {
          distances_from_targets[index] = (position - targets->at(index).position).Length();
          mean_distance_from_targets += distances_from_targets[index];
          max_distance_from_targets = std::max(max_distance_from_targets, distances_from_targets[index]);
        }
        mean_distance_from_targets /= targets->size();
        state = State::VOTING;
      }
      ++voting_sessions;
    }

    void Voting() {
      std::vector<prez::Target>* targets = prez::GetTargetList();
      /** formations is a X:Y map. It has to be thought as:
       * at this time we sense Y drones belonging to the X target */
      std::unordered_map<uint32_t, uint32_t> formations;
      /** with rab we sense all other drones in the arena */
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings(); 
      for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours) {
        ++formations[neighbour.Data[prez::RABKey::TARGET]];
        }
      /** we belong to the target "target" */
      ++formations[task->target];

      //the current target has enough drones assigend to it?
      if (formations[task->target] >= targets->at(task->target).force) {
        for (auto formation : formations) {
          if (formation.first != task->target) {
            //this target has not enough drones assigend to it?
            if (formation.second < targets->at(formation.first).force) {
              /** we want a reassignment_probability to this target
               *  that scale with the inverse of the distance to it (normalized) */
              double_t reassignment_probability = 1 - (distances_from_targets[formation.first] / max_distance_from_targets);
              bool reassign = random_number_generator->Bernoulli(reassignment_probability);
              if (reassign) {
                task->target = formation.first;
                /** we changed our formation, so our voting is over. */
                break;
              }
            }
          }
        }
      }

      ++voting_sessions;
      if (voting_sessions > MAX_VOTING_SESSIONS) {
        state = State::IDLE;
      } else {
        ++voting_sessions;
      }
    }

    void Idle() {}

    void Round() {
      switch(state) {
        case State::START: Start(); break;
        case State::VOTING: Voting(); break;
        case State::IDLE: Idle(); break;
      }
    }

    void Reset() {
      state = START;
      voting_sessions = 0;
      distances_from_targets.clear();
      max_distance_from_targets = 0.0f;
      mean_distance_from_targets = 0.0f;
    }
  };
}
#endif//TASK_ALLOCATORS_DEFAULT_HH
