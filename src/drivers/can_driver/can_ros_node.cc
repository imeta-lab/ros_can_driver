#include "can_ros.h"

#include "common/log.h"

#include <stdio.h>

using namespace humanoid::can_driver;

int main(int argc, char** argv) {
  ros::init(argc, argv, "can_driver");

  // init glog
  google::InitGoogleLogging(argv[0]);
  FLAGS_minloglevel = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_alsologtostderr = true;

  std::string log_dir =
      "/home/ubuntu/humanoid_robot/HumanoidRobot/log/can_driver/";
  if (access(log_dir.c_str(), F_OK) != 0) {
    std::string command = "mkdir -p " + log_dir;

    // Open a pipe to execute the command
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
      AERROR << "Error executing command: " << command << std::endl;
      return -1;
    }

    // Close the pipe
    pclose(pipe);
  }
  FLAGS_log_dir = log_dir;
  AINFO << "can_driver log dir: " << FLAGS_log_dir;

  // can driver ros
  CanRos can_driver_ros;
  // init
  if (!can_driver_ros.Init()) {
    AERROR << "Failed to init Can Driver module";

    return -1;
  }

  // loop
  can_driver_ros.Process();

  return 0;
}