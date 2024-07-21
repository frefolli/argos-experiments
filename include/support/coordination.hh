#ifndef COORDINATION_HH
#define COORDINATION_HH
/** @file coordination.hh */
#include <mutex>

namespace prez {
  struct Coordination {
    public:
      static inline Coordination& GetInstance() {
        return *__ISTANCE;
      }

      void Finished();

    private:
      std::mutex mutex;
      uint32_t finished_counter = 0;
      static Coordination* __ISTANCE;
  };
}
#endif//COORDINATION_HH
