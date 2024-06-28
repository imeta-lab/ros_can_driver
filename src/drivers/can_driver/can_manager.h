#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "proto/can_driver_config.pb.h"

#include "can_interface_base/can_reader_base.h"
#include "can_interface_base/can_writer_base.h"
#include "common/motor_control_command.h"
#include "common/motor_states.h"

namespace humanoid {
namespace can_driver {

class CanManager {
 public:
  CanManager() = delete;

  explicit CanManager(const CanDriverConfig& config);

  ~CanManager();

  bool Init();

  void RunOnce(const ControlCommandVector& control_command,
               MotorStateVector& motor_states_info);

 private:
  bool OpenCanDevice(const std::string& can_device_id, int& socket);

  void GenerateThread(
      const std::vector<std::shared_ptr<CanReaderBase>>& readers, int socket,
      std::shared_ptr<std::mutex> mtx);

  bool WriteCanFrame(const can_frame& frame, int socket) const;

 private:
  // socket can
  std::unordered_map<std::string, int> socket_can_map_;

  // reader threads
  std::vector<std::thread> reader_threads_;
  std::vector<std::shared_ptr<std::mutex>> readers_mutux_;
  std::atomic<bool> stop_flag_ = false;

  // readers and writers
  std::vector<std::vector<std::shared_ptr<CanReaderBase>>> can_readers_;
  std::vector<std::unique_ptr<CanWriterBase>> can_writers_;

  // config
  CanDriverConfig config_;
};

}  // namespace can_driver
}  // namespace humanoid