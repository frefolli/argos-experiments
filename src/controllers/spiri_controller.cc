#include <argos3/core/utility/configuration/tinyxml/ticpp.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_speed_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <controllers/spiri_controller.hh>
#include <support/targets.hh>
#include <support/vectors.hh>
#include <support/logging.hh>
#include <cmath>
#include <string>
#include <cassert>

prez::SpiriController::SpiriController() :
  range_and_bearing_actuator(nullptr)
  {}

void prez::SpiriController::Init(argos::TConfigurationNode& /* t_node */) {
  range_and_bearing_actuator = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing");

  task_executor.speed_actuator = GetActuator<argos::CCI_QuadRotorSpeedActuator>("quadrotor_speed");
  task_executor.range_and_bearing_sensor = GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing");
  task_executor.positioning_sensor = GetSensor<argos::CCI_PositioningSensor>("positioning");
  task_executor.task = &task;

  task_allocator.range_and_bearing_sensor = GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing");
  task_allocator.positioning_sensor = GetSensor<argos::CCI_PositioningSensor>("positioning");
  task_allocator.random_number_generator = argos::CRandom::CreateRNG("argos");
  task_allocator.task = &task;
  task_allocator.Init();

  logfile.open(prez::DroneLogfile(GetId()));
  logfile
    << "Timestamp,PosX,PosY,PosZ,Target,DistanceFromTarget,Speed,TaskAllocatorState,TaskExecutorState"
    << std::endl;
  Reset();
}

void prez::SpiriController::Destroy() {
  logfile.close();
}

void prez::SpiriController::ControlStep() {
  task_allocator.Round();
  task_executor.Round();
  /*we broadcast this (updated) infos with the rab
  */
  range_and_bearing_actuator->SetData(prez::RABKey::ID, task.id);
  range_and_bearing_actuator->SetData(prez::RABKey::TARGET, task.target);
  range_and_bearing_actuator->SetData(prez::RABKey::TASK_ALLOCATOR_STATE, task_allocator.state);
  range_and_bearing_actuator->SetData(prez::RABKey::TASK_EXECUTOR_STATE, task_executor.state);

  // if (task_executor.state != decltype(task_executor)::State::ARRIVED)
    logfile
      << task_executor.tick << ","
      << task_executor.positioning_sensor->GetReading().Position << ","
      << task.target << ","
      << task.distance_from_target << ","
      << task.speed << ","
      << task_allocator.state << ","
      << task_executor.state
      << std::endl;
}

void prez::SpiriController::Reset() {
  task.id = std::stoi(GetId().substr(2));
  task.target = -1;
  task_allocator.Reset();
  task_executor.Reset();
}

using namespace prez;
REGISTER_CONTROLLER(SpiriController, "spiri_controller")
