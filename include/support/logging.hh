#ifndef LOGGING_HH
#define LOGGING_HH
#include <string>
namespace prez {
  inline std::string DroneLogfile(std::string ID) {
    return "./out/drones/" + ID + ".csv";
  }

  inline std::string TargetsLogfile() {
    return "./out/targets.csv";
  }
}
#endif//LOGGING_HH
