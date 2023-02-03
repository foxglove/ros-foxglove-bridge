#pragma once

#include <any>
#include <stdint.h>
#include <string>
#include <vector>

namespace foxglove {

enum class ParameterSubscriptionOperation {
  SUBSCRIBE,
  UNSUBSCRIBE,
};

enum class ParameterType {
  PARAMETER_NOT_SET,
  PARAMETER_BOOL,
  PARAMETER_INTEGER,
  PARAMETER_DOUBLE,
  PARAMETER_STRING,
  PARAMETER_BOOL_ARRAY,
  PARAMETER_INTEGER_ARRAY,
  PARAMETER_DOUBLE_ARRAY,
  PARAMETER_STRING_ARRAY,
};

class Parameter {
public:
  Parameter();
  Parameter(const std::string& name);
  Parameter(const std::string& name, bool value);
  Parameter(const std::string& name, int value);
  Parameter(const std::string& name, int64_t value);
  Parameter(const std::string& name, double value);
  Parameter(const std::string& name, std::string value);
  Parameter(const std::string& name, const char* value);
  Parameter(const std::string& name, const std::vector<bool>& value);
  Parameter(const std::string& name, const std::vector<int>& value);
  Parameter(const std::string& name, const std::vector<int64_t>& value);
  Parameter(const std::string& name, const std::vector<double>& value);
  Parameter(const std::string& name, const std::vector<std::string>& value);

  inline const std::string& getName() const {
    return _name;
  }

  inline ParameterType getType() const {
    return _type;
  }

  template <typename T>
  inline const T& getValue() const {
    return std::any_cast<const T&>(_value);
  }

private:
  std::string _name;
  ParameterType _type;
  std::any _value;
};

}  // namespace foxglove
