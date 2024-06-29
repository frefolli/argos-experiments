#ifndef TASK_ALLOCATORS_NN_HH
#define TASK_ALLOCATORS_NN_HH
/** @file NN.hh */
#include <limits>
#include <support/task_allocator.hh>
#include <support/squadrons.hh>
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
    argos::CRandom::CRNG* random_number_generator;//not used here but in spiri_controller
    argos::CCI_PositioningSensor* positioning_sensor;//not used here but in spiri_controller
    argos::CCI_RangeAndBearingSensor* range_and_bearing_sensor;//not used here but in spiri_controller

    
    std::unordered_map<uint32_t, double_t> distances_from_squadrons;
    double_t min_distance_from_squadrons = std::numeric_limits<int>::max();
    double_t nearest_squadron;

    void Start() {
      std::vector<prez::Squadron>* squadrons = prez::GetSquadronList();
      argos::CVector3 position = positioning_sensor->GetReading().Position;

      for (uint32_t index = 0; index < squadrons->size(); ++index) {
          distances_from_squadrons[index] = (position - squadrons->at(index).position).Length();
          if(distances_from_squadrons[index] < min_distance_from_squadrons){
            min_distance_from_squadrons = distances_from_squadrons[index];
            nearest_squadron = index;
          }          
      }
      task->squadron = nearest_squadron;//my squadron is the nearest to me
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
      distances_from_squadrons.clear();
      min_distance_from_squadrons = std::numeric_limits<int>::max();
    }
  };
}
#endif//TASK_ALLOCATORS_NN_HH
