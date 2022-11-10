#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <rospack/rospack.h>

#include <foxglove_bridge/msg_parser.hpp>

namespace foxglove_bridge {

constexpr char EXT[] = ".msg";
constexpr char SUB_PATH[] = "msg";
constexpr std::array PRIMITIVE_TYPES = {
  "int8",    "uint8",   "int16",  "uint16", "int32", "uint32",   "int64", "uint64",
  "float32", "float64", "string", "bool",   "time",  "duration", "char",  "byte"};
constexpr char HEADER[] = "std_msgs/Header";
constexpr std::array HEADER_TYPES = {"Header", "std_msgs/Header", "roslib/Header"};

static bool is_builtin_type(const std::string& type_name) {
  for (std::size_t i = 0; i < std::size(PRIMITIVE_TYPES); ++i) {
    if (type_name == PRIMITIVE_TYPES[i]) {
      return true;
    }
  }
  return false;
}

static bool is_header_type(const std::string& type_name) {
  for (std::size_t i = 0; i < std::size(HEADER_TYPES); ++i) {
    if (type_name == HEADER_TYPES[i]) {
      return true;
    }
  }
  return false;
}

static std::pair<std::string, std::string> get_package_and_message_name(
  const std::string& message_type) {
  std::vector<std::string> split_str;
  boost::algorithm::split(split_str, message_type, boost::algorithm::is_any_of("/"));
  if (split_str.size() != 2) {
    throw InvalidMessageType("Invalid message type " + message_type);
  }

  const std::string& package_name = split_str[0];
  const std::string& message_name = split_str[1];
  return {package_name, message_name};
}

MsgParser::MsgParser(std::shared_ptr<rospack::Rosstackage> rospack)
    : rospack_(rospack ? rospack : std::make_shared<rospack::Rospack>()) {
  if (!rospack) {
    // Perform a crawl if rospack was not passed in as argument.
    std::vector<std::string> search_path;
    if (rospack_->getSearchPathFromEnv(search_path)) {
      const bool force_crawl = true;
      rospack_->crawl(search_path, force_crawl);
    }
  }
}

std::string MsgParser::get_msg_file_path(const std::string& package_name,
                                         const std::string& message_name) {
  std::string package_path;
  if (rospack_->find(package_name, package_path)) {
    const auto msg_path = boost::filesystem::path(package_path) / SUB_PATH / (message_name + EXT);
    if (boost::filesystem::exists(msg_path)) {
      return msg_path.string();
    }
  }

  throw MsgNotFoundException("Unable to find message definition " + message_name + " in package " +
                             package_name);
}

MsgSpec MsgParser::parse_msg_file(const std::string& package_name, const std::string& file_path) {
  MsgSpec spec;
  std::ifstream infile(file_path);
  std::string line;
  while (std::getline(infile, line)) {
    spec.full_text += line + "\n";
    line = line.substr(0, line.find("#"));     // Remove comments.
    line = boost::algorithm::trim_copy(line);  // Trim

    if (line.empty()) {
      // Ignore empty lines.
      continue;
    } else if (line.find("=") != std::string::npos) {
      // Ignore constants.
      continue;
    } else {
      // Field declaration.
      std::vector<std::string> split_str;
      boost::algorithm::split(split_str, line, boost::algorithm::is_any_of(" "));
      if (split_str.size() < 2) {
        throw MsgSpecException("Invalid declaration: " + line);
      }

      std::string field_type = split_str.front();
      // We don't care if it is an array type.
      field_type = field_type.substr(0, field_type.find("["));

      if (is_builtin_type(field_type)) {
        continue;
      } else if (is_header_type(field_type)) {
        spec.types.push_back(HEADER);
      } else if (field_type.find("/") == std::string::npos) {
        // Message type specifier without package_name. References messages in same package.
        spec.types.push_back(package_name + "/" + field_type);
      } else {
        // Explicit type specifier: <package_name>/<message_name>.
        spec.types.push_back(field_type);
      }
    }
  }
  return spec;
}

std::string MsgParser::get_message_schema(const std::string& message_type) {
  std::vector<std::string> subtypes;
  std::list<std::string> type_list = {message_type};

  while (!type_list.empty()) {
    const auto type = type_list.front();

    const auto [package_name, message_name] = get_package_and_message_name(type);
    if (msg_spec_cache_.find(type) == msg_spec_cache_.end()) {
      msg_spec_cache_.insert(
        {type, parse_msg_file(package_name, get_msg_file_path(package_name, message_name))});
    }

    const auto spec = msg_spec_cache_.at(type);
    for (const auto& subtype : spec.types) {
      if (std::find(subtypes.begin(), subtypes.end(), subtype) == subtypes.end()) {
        subtypes.push_back(subtype);
      }
      type_list.push_back(subtype);
    };
    type_list.pop_front();
  }

  // Construct string of concatenated message definitions.
  std::string ret_string = msg_spec_cache_[message_type].full_text;
  for (const auto& type : subtypes) {
    ret_string += "\n" + std::string(80, '=') + "\n";
    ret_string += "MSG: " + type + "\n";
    ret_string += msg_spec_cache_[type].full_text;
  }

  return ret_string;
}

}  // namespace foxglove_bridge
