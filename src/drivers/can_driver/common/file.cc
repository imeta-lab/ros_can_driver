#include "common/file.h"

namespace humanoid {
namespace common {

bool GetProtoFromFile(const std::string& file_name,
                      google::protobuf::Message* message) {
  if (!PathExists(file_name)) {
    AERROR << "File [" << file_name << "] does not exists!";
    return false;
  }

  // Try the binary parser first if it's much likely a binary proto.
  std::string kBinExt = ".bin";
  if (std::equal(kBinExt.rbegin(), kBinExt.rend(), file_name.rbegin())) {
    return GetProtoFromBinaryFile(file_name, message) ||
           GetProtoFromASCIIFile(file_name, message);
  }

  return GetProtoFromASCIIFile(file_name, message) ||
         GetProtoFromBinaryFile(file_name, message);
}

// parse binary file
bool GetProtoFromBinaryFile(const std::string& file_name,
                            google::protobuf::Message* message) {
  std::fstream input(file_name, std::ios::in | std::ios::binary);
  if (!input.good()) {
    AERROR << "Failed to open file " << file_name << " in binary mode.";
    return false;
  }

  if (!message->ParseFromIstream(&input)) {
    AERROR << "Failed to parse file " << file_name << "as binary proto.";
    return false;
  }

  return true;
}

// parse pb.txt file
bool GetProtoFromASCIIFile(const std::string& file_name,
                           google::protobuf::Message* message) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    // Failed open file
    AERROR << "Failed to open file " << file_name << " in text mode.";
    return false;
  }

  google::protobuf::io::ZeroCopyInputStream* input =
      new google::protobuf::io::FileInputStream(file_descriptor);
  bool success = google::protobuf::TextFormat::Parse(input, message);
  if (!success) {
    AERROR << "Failed to parse file " << file_name << " as  text proto.";
  }

  delete input;
  input = nullptr;
  close(file_descriptor);

  return success;
}

// stat function in <sys/stat.h>
bool PathExists(const std::string& path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

}  // namespace common
}  // namespace humanoid