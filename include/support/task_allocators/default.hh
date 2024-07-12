#ifndef TASK_ALLOCATORS_DEFAULT_HH
#define TASK_ALLOCATORS_DEFAULT_HH
/** @file default.hh */
#include <cstdlib>
#include <cstring>
#include <support/task_allocator.hh>
#include <support/targets.hh>
#include <support/rab.hh>
#include <support/task.hh>
#include <support/setup.hh>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/rng.h>

namespace prez::task_allocators
{
  class Default : public TaskAllocator
  {
    static constexpr uint32_t MAX_REVIEWING_SESSIONS = 20;

  public:
    enum InitialChoiceStrategy
    {
      RANDOM, /* Initial target is random */
      NEAREST /* Initial target is nearest within Euclidean Distance */
    } initial_choice_strategy;

    enum ReviewChoiceStrategy
    {
      NO_REVIEW,                      /* No Review Phase */
      ALWAYS_RANDOM_WHEN_IN_EXCESS,   /* Always jump to a random target (!= current) when this formation is in excess of force */
      PROBABLE_RANDOM_WHEN_IN_EXCESS, /* May jump to a random target (!= current) when this formation is in excess of force */
    } review_choice_strategy;
    double probability_of_change_in_review;

    enum State
    {
      START,
      REVIEWING,
      IDLE
    } state = START;
    uint32_t reviewing_sessions = 0;

    std::unordered_map<uint32_t, double_t> distances_from_targets;
    double_t max_distance_from_targets;
    double_t min_distance_from_targets = std::numeric_limits<int>::max();

    Task *task;
    argos::CRandom::CRNG *random_number_generator;
    argos::CCI_PositioningSensor *positioning_sensor;
    argos::CCI_RangeAndBearingSensor *range_and_bearing_sensor;

    inline void ParseInitialChoiceStrategy()
    {
      initial_choice_strategy = RANDOM;
      char *strategy = std::getenv("INITIAL_CHOICE_STRATEGY");
      if (strategy != nullptr)
      {
        PARSE_ENV_SETUP(initial_choice_strategy, RANDOM);
        PARSE_ENV_SETUP(initial_choice_strategy, NEAREST);
      }
      // std::cout << "Using INITIAL_CHOICE_STRATEGY = " << initial_choice_strategy << std::endl;
    }

    inline void ParseReviewChoiceStrategy()
    {
      review_choice_strategy = NO_REVIEW;
      char *strategy = std::getenv("REVIEW_CHOICE_STRATEGY");
      if (strategy != nullptr)
      {
        PARSE_ENV_SETUP(review_choice_strategy, NO_REVIEW);
        PARSE_ENV_SETUP(review_choice_strategy, ALWAYS_RANDOM_WHEN_IN_EXCESS);
        PARSE_ENV_SETUP(review_choice_strategy, PROBABLE_RANDOM_WHEN_IN_EXCESS);
      }
      switch (review_choice_strategy)
      {
      case ALWAYS_RANDOM_WHEN_IN_EXCESS:
        probability_of_change_in_review = 1.0f;
        break;
      case PROBABLE_RANDOM_WHEN_IN_EXCESS:
        probability_of_change_in_review = 0.3f;
        break;
      case NO_REVIEW:
        break;
      }
      // std::cout << "Using REVIEW_CHOICE_STRATEGY = " << review_choice_strategy << std::endl;
    }

    void Init()
    {
      ParseInitialChoiceStrategy();
      ParseReviewChoiceStrategy();
    }

    void Start()
    {
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      uint32_t const n = argos::CSimulator::GetInstance().GetSpace().GetEntitiesByType("eye-bot").size();
      assert(n == neighbours.size() + 1); // we are able to reach all other drones with our rab

      std::vector<prez::Target> *targets = prez::GetTargetList();
      if (targets->empty())
      {
        std::cerr << "Warning: targets is empty" << std::endl;
      }
      else
      {
        switch (initial_choice_strategy)
        {
        case InitialChoiceStrategy::RANDOM:
        {
          argos::CRange<uint32_t> target_range(0, targets->size());
          task->target = random_number_generator->Uniform(target_range);
        };
        break;
        case InitialChoiceStrategy::NEAREST:
        {
          uint32_t nearest_target;
          max_distance_from_targets = 0.0f;
          argos::CVector3 position = positioning_sensor->GetReading().Position;
          for (uint32_t index = 0; index < targets->size(); ++index)
          {
            distances_from_targets[index] = (position - targets->at(index).position).Length();
            max_distance_from_targets = std::max(max_distance_from_targets, distances_from_targets[index]);
            if (distances_from_targets[index] < min_distance_from_targets)
            {
              min_distance_from_targets = distances_from_targets[index];
              nearest_target = index;
            }
          }
          task->target = nearest_target; // my target is the nearest to me
        };
        break;
        }
        state = State::REVIEWING;
        ++reviewing_sessions;
      }
    }

    void Review()
    {
      std::vector<prez::Target> *targets = prez::GetTargetList();
      /** formations is a X:Y map. It has to be thought as:
       * at this time we sense Y drones belonging to the X target */
      std::unordered_map<uint32_t, uint32_t> formations;
      /** with rab we sense all other drones in the arena */
      const argos::CCI_RangeAndBearingSensor::TReadings neighbours = range_and_bearing_sensor->GetReadings();
      for (argos::CCI_RangeAndBearingSensor::SPacket neighbour : neighbours)
      {
        ++formations[neighbour.Data[prez::RABKey::TARGET]];
      }
      /** we belong to the target "target" */
      ++formations[task->target];

      switch (review_choice_strategy)
      {
      case ReviewChoiceStrategy::NO_REVIEW:
      {
      };
      break;
      case ReviewChoiceStrategy::ALWAYS_RANDOM_WHEN_IN_EXCESS:
      case ReviewChoiceStrategy::PROBABLE_RANDOM_WHEN_IN_EXCESS:
      {
        // if the current target has enough drones assigend to it...we relocate
        if (formations[task->target] > targets->at(task->target).force && random_number_generator->Bernoulli() <= probability_of_change_in_review)
        {
          argos::CRange<uint32_t> target_range(0, targets->size());
          uint32_t temp = task->target;
          for (; temp == task->target;) // we want to avoid to join the current squadron
          {
            temp = random_number_generator->Uniform(target_range);
          }
          task->target = temp;
        }
      };
      break;
      }

      ++reviewing_sessions;
      if (reviewing_sessions > MAX_REVIEWING_SESSIONS)
      {
        state = State::IDLE;
      }
      else
      {
        ++reviewing_sessions;
      }
    }

    void Idle() {}

    void Round()
    {
      switch (state)
      {
      case State::START:
        Start();
        break;
      case State::REVIEWING:
        Review();
        break;
      case State::IDLE:
        Idle();
        break;
      }
    }

    void Reset()
    {
      state = START;
      reviewing_sessions = 0;
      distances_from_targets.clear();
      max_distance_from_targets = 0.0f;
    }
  };
}
#endif // TASK_ALLOCATORS_DEFAULT_HH
