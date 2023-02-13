#include <foxglove_bridge/parameter.hpp>

namespace foxglove {

Parameter::Parameter()
    : _name("")
    , _type(ParameterType::PARAMETER_NOT_SET)
    , _value() {}

Parameter::Parameter(const std::string& name)
    : _name(name)
    , _type(ParameterType::PARAMETER_NOT_SET)
    , _value() {}

Parameter::Parameter(const std::string& name, bool value)
    : _name(name)
    , _type(ParameterType::PARAMETER_BOOL)
    , _value(value) {}

Parameter::Parameter(const std::string& name, int value)
    : Parameter(name, static_cast<int64_t>(value)) {}

Parameter::Parameter(const std::string& name, int64_t value)
    : _name(name)
    , _type(ParameterType::PARAMETER_INTEGER)
    , _value(value) {}

Parameter::Parameter(const std::string& name, double value)
    : _name(name)
    , _type(ParameterType::PARAMETER_DOUBLE)
    , _value(value) {}

Parameter::Parameter(const std::string& name, const char* value)
    : Parameter(name, std::string(value)) {}

Parameter::Parameter(const std::string& name, std::string value)
    : _name(name)
    , _type(ParameterType::PARAMETER_STRING)
    , _value(value) {}

Parameter::Parameter(const std::string& name, const std::vector<bool>& value)
    : _name(name)
    , _type(ParameterType::PARAMETER_BOOL_ARRAY)
    , _value(value) {}

Parameter::Parameter(const std::string& name, const std::vector<int>& value)
    : _name(name)
    , _type(ParameterType::PARAMETER_INTEGER_ARRAY)
    , _value(std::vector<int64_t>(value.begin(), value.end())) {}

Parameter::Parameter(const std::string& name, const std::vector<int64_t>& value)
    : _name(name)
    , _type(ParameterType::PARAMETER_INTEGER_ARRAY)
    , _value(value) {}

Parameter::Parameter(const std::string& name, const std::vector<double>& value)
    : _name(name)
    , _type(ParameterType::PARAMETER_DOUBLE_ARRAY)
    , _value(value) {}

Parameter::Parameter(const std::string& name, const std::vector<std::string>& value)
    : _name(name)
    , _type(ParameterType::PARAMETER_STRING_ARRAY)
    , _value(value) {}

}  // namespace foxglove
