#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace rospack {
class Rosstackage;
}

namespace foxglove_bridge {

struct MsgSpecException : public std::runtime_error {
  using std::runtime_error::runtime_error;
};
struct MsgNotFoundException : public std::runtime_error {
  using std::runtime_error::runtime_error;
};
struct InvalidMessageType : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct MsgSpec {
  /// @brief Raw message definition.
  std::string full_text;
  /// @brief Field types.
  std::vector<std::string> types;
};

class MsgParser {
public:
  explicit MsgParser(std::shared_ptr<rospack::Rosstackage> rospack = nullptr);

  /// @brief Get the full message definition including definitions of all subtypes.
  /// The output is equal to what `rosrun roslib gendeps -c <msg_file>` generates.
  /// @param message_type Message type in the format <package_name>/<message_name>
  /// @return String with the message schema and concatenated schemas of submessages.
  std::string get_message_schema(const std::string& message_type);

private:
  std::string get_msg_file_path(const std::string& package_name, const std::string& message_name);
  MsgSpec parse_msg_file(const std::string& package_name, const std::string& file_path);
  std::shared_ptr<rospack::Rosstackage> rospack_;
  std::unordered_map<std::string, MsgSpec> msg_spec_cache_;
};

}  // namespace foxglove_bridge
