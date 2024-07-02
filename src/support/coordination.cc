#include <mutex>
#include <support/coordination.hh>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>

prez::Coordination* prez::Coordination::__ISTANCE = new prez::Coordination();


void prez::Coordination::Finished() {
  std::scoped_lock lock(mutex);
  ++finished_counter;

  const uint32_t number_of_eyebots = argos::CSimulator::GetInstance().GetSpace().GetEntitiesByType("eye-bot").size();
  if (number_of_eyebots == finished_counter) {
    argos::CSimulator::GetInstance().Terminate();
  }
}
