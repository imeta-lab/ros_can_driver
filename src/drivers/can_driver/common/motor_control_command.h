#pragma once

#include <vector>

namespace humanoid {
namespace can_driver {

struct MitControlMode {
  double position;
  double velocity;
  double kp;
  double kd;
  double torque;
};

using ControlCommandVector = std::vector<MitControlMode>;

}  // namespace can_driver
}  // namespace humanoid