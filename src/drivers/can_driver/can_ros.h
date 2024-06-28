#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "can_manager.h"
#include "common/motor_control_command.h"
#include "common/motor_states.h"

#include "proto/can_driver_config.pb.h"
#include "ros_msgs/ControlCommand.h"
#include "ros_msgs/MotorStateArray.h"

namespace humanoid {
namespace can_driver {

class CanRos {
 public:
  CanRos() : nh_private_("~") {}

  ~CanRos() = default;

  bool Init();

  void Process();

 private:
  // init parameters from roslaunch
  bool InitRosParam();

  // init reader、writer...
  void InitTopic();

  // control command from control module
  void ControlCommandCallBack(const ros_msgs::ControlCommand& msg);

  // convert struct MotorStateVector to rosmsg MotorStateArray
  void ConvertToRosmsg(const MotorStateVector& motor_states_info,
                       ros_msgs::MotorStateArray& motor_states_msg);

 private:
  // private namespace
  ros::NodeHandle nh_private_;
  // reader
  ros::Subscriber control_command_reader_;
  // writer
  ros::Publisher motor_state_writer_;

  // control command
  ControlCommandVector control_command_;

  std::unique_ptr<CanManager> can_manager_;

  // config
  std::string config_path_;
  CanDriverConfig config_;
};

}  // namespace can_driver
}  // namespace humanoid