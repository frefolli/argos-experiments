#ifndef TASK_HH
#define TASK_HH
/** @file task.hh */
#include <cmath>
#include <cstdint>
#include <argos3/core/utility/math/vector3.h>

namespace prez {
  struct Task {
    uint32_t id;
    uint32_t target;
    double_t speed;
    argos::CVector3 target_direction;
    double_t distance_from_target;
  };
}
#endif//TASK_HH
