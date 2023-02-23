#pragma once

#include <algorithm>
#include <regex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace foxglove_bridge {

inline std::pair<std::string, std::string> getNodeAndNodeNamespace(const std::string& fqnNodeName) {
  const std::size_t found = fqnNodeName.find_last_of("/");
  if (found == std::string::npos) {
    throw std::runtime_error("Invalid fully qualified node name: " + fqnNodeName);
  }
  return std::make_pair(fqnNodeName.substr(0, found), fqnNodeName.substr(found + 1));
}

inline bool isWhitelisted(const std::string& name, const std::vector<std::regex>& regexPatterns) {
  return std::find_if(regexPatterns.begin(), regexPatterns.end(), [name](const auto& regex) {
           return std::regex_match(name, regex);
         }) != regexPatterns.end();
}

}  // namespace foxglove_bridge
