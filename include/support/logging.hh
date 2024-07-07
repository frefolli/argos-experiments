#ifndef LOGGING_HH
#define LOGGING_HH
#include <string>
#include <filesystem>
namespace prez {
  #define DefineLogDir(methodName, path) \
    inline std::string methodName() { \
      std::string dir = path; \
      std::filesystem::create_directories(path); \
      return dir; \
    }

  DefineLogDir(DroneLogDir, "./out/drones/");
  DefineLogDir(TargetsLogDir, "./out/targets/");
  DefineLogDir(LogDir, "./out/");

  inline std::string DroneLogfile(std::string ID) {
    return DroneLogDir() + ID + ".csv";
  }

  inline std::string TargetsLogfile() {
    return LogDir() + "targets.csv";
  }
}
#endif//LOGGING_HH
