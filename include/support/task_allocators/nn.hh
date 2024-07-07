#ifndef TASK_ALLOCATORS_NN_HH
#define TASK_ALLOCATORS_NN_HH
/** @file NN.hh */
#include <limits>
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
  class NN : public TaskAllocator {

    public:
    enum State {
      START, IDLE
    } state = START;

    Task* task;
    argos::CRandom::CRNG* random_number_generator;//not used here but every task_allocators should have it
    argos::CCI_PositioningSensor* positioning_sensor;
    argos::CCI_RangeAndBearingSensor* range_and_bearing_sensor;//not used here but every task_allocators should have it

    
    std::unordered_map<uint32_t, double_t> distances_from_targets;
    double_t min_distance_from_targets = std::numeric_limits<int>::max();
    double_t nearest_target;

    void Start() {
      std::vector<prez::Target>* targets = prez::GetTargetList();
      argos::CVector3 position = positioning_sensor->GetReading().Position;

      for (uint32_t index = 0; index < targets->size(); ++index) {
          distances_from_targets[index] = (position - targets->at(index).position).Length();
          if(distances_from_targets[index] < min_distance_from_targets){
            min_distance_from_targets = distances_from_targets[index];
            nearest_target = index;
          }          
      }
      task->target = nearest_target;//my target is the nearest to me
      state = State::IDLE;
    }

    void Idle() {}

    void Round() {
      switch(state) {
        case State::START: Start(); break;
        case State::IDLE: Idle(); break;
      }
    }

    void Reset() {
      state = START;
      distances_from_targets.clear();
      min_distance_from_targets = std::numeric_limits<int>::max();
    }
  };
}
#endif//TASK_ALLOCATORS_NN_HH
