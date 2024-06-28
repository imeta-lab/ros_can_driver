#pragma once

#include <fcntl.h>
#include <sys/stat.h>
#include <fstream>
#include <string>

#include "common/log.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

namespace humanoid {
namespace common {

/**
 * @brief Parses the content of the file specified by the file_name,
          and merge the parsed content to proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
bool GetProtoFromFile(const std::string& file_name,
                      google::protobuf::Message* message);

/**
 * @brief Parses the content of the file specifies by the file_name as a
          binary representation of protobufs.(binary file)
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
bool GetProtoFromBinaryFile(const std::string& file_name,
                            google::protobuf::Message* message);

/**
 * @brief Parses the content of the file specifies by the file_name as a
          ascii representation of protobufs.(proto.pb.txt)
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
bool GetProtoFromASCIIFile(const std::string& file_name,
                           google::protobuf::Message* message);

/**
 * @brief Check if the path exists.
 * @param path file absolute path
 * @return If the path exists.
 */
bool PathExists(const std::string& path);

}  // namespace common
}  // namespace humanoid