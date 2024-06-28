#pragma once

#include <vector>

namespace humanoid {
namespace can_driver {

struct MotorState {
  double position;
  double velocity;
  double torque;
};

using MotorStateVector = std::vector<MotorState>;

}  // namespace can_driver
}  // namespace humanoid