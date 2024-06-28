#include "can_ros.h"

#include "common/file.h"
#include "common/log.h"

#include "ros_msgs/MotorStateArray.h"

namespace humanoid {
namespace can_driver {

bool CanRos::Init() {
  if (!InitRosParam()) {
    AERROR << "Failed to init can_driver ros param";
    return false;
  }

  // load config
  ACHECK(common::GetProtoFromFile(config_path_, &config_))
      << "Failed to load config from: " << config_path_;
  AINFO << "Success to load can_driver_config from: " << config_path_;

  can_manager_ = std::make_unique<CanManager>(config_);
  if (!can_manager_->Init()) {
    AERROR << "Failed to init Class CanManager";
    return false;
  }

  InitTopic();

  return true;
}

bool CanRos::InitRosParam() {
  if (!nh_private_.getParam("config_path", config_path_)) {
    AERROR << "Failed get private param [config_path]";
    return false;
  }

  // add other ros param here

  return true;
}

void CanRos::InitTopic() {
  // Reader
  control_command_reader_ =
      nh_private_.subscribe(config_.topic_config().control_command_topic(), 1,
                            &CanRos::ControlCommandCallBack, this);

  // Writer
  motor_state_writer_ = nh_private_.advertise<ros_msgs::MotorStateArray>(
      config_.topic_config().motor_state_topic(), 1, true);
}

void CanRos::ControlCommandCallBack(const ros_msgs::ControlCommand& msg) {
  if (msg.control_command.empty()) {
    AERROR << "Control Command is empty";
    return;
  }

  control_command_.clear();

  // convert rosmsg to struct
  MitControlMode motor_control;
  for (int i = 0; i < msg.control_command.size(); i++) {
    motor_control.position = msg.control_command.at(i).p;
    motor_control.velocity = msg.control_command.at(i).dp;
    motor_control.kp = msg.control_command.at(i).kp;
    motor_control.kd = msg.control_command.at(i).kd;
    motor_control.torque = msg.control_command.at(i).torque;

    control_command_.push_back(motor_control);
  }
}

void CanRos::ConvertToRosmsg(const MotorStateVector& motor_states_info,
                             ros_msgs::MotorStateArray& motor_states_msg) {
  motor_states_msg.motor_state.clear();

  ros_msgs::MotorState motor_state;
  for (int i = 0; i < motor_states_info.size(); i++) {
    motor_state.p = motor_states_info.at(i).position;
    motor_state.dp = motor_states_info.at(i).velocity;
    motor_state.torque = motor_states_info.at(i).torque;

    motor_states_msg.motor_state.emplace_back(motor_state);
  }
}

void CanRos::Process() {
  ros::Rate loop_rate(config_.rate());
  while (ros::ok()) {
    ros::spinOnce();

    if (control_command_.empty()) {
      // AINFO << "No Control Command";

      loop_rate.sleep();
      continue;
    }

    // run once
    MotorStateVector motor_states_info;
    can_manager_->RunOnce(control_command_, motor_states_info);

    // convert struct MotorStateVector to rosmsg MotorStateArray
    ros_msgs::MotorStateArray motor_states_msg;
    ConvertToRosmsg(motor_states_info, motor_states_msg);

    // publish motor states
    motor_states_msg.header.frame_id = "map";
    motor_states_msg.header.stamp = ros::Time::now();
    motor_state_writer_.publish(motor_states_msg);

    loop_rate.sleep();
  }
}

}  // namespace can_driver
}  // namespace humanoid