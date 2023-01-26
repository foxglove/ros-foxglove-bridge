#pragma once

#include <future>
#include <string>
#include <vector>

#include <ros/serialization.h>
#include <ros/service_traits.h>

namespace foxglove_bridge {

struct GenericServiceType {
  std::string type;
  std::string md5sum;
  std::vector<uint8_t> data;

  template <typename Stream>
  void write(Stream& stream) const {
    std::memcpy(stream.getData(), data.data(), data.size());
  }

  template <typename Stream>
  void read(Stream& stream) {
    data.resize(stream.getLength());
    std::memcpy(data.data(), stream.getData(), stream.getLength());
  }
};

/**
 * Opens a socket to the service server and retrieves the service type from the
 * connection header. The service type is not stored on the ROS master.
 */
std::future<std::string> retrieveServiceType(const std::string& serviceName);

}  // namespace foxglove_bridge

namespace ros::service_traits {
template <>
struct MD5Sum<foxglove_bridge::GenericServiceType> {
  static const char* value(const foxglove_bridge::GenericServiceType& m) {
    return m.md5sum.c_str();
  }

  static const char* value() {
    return "*";
  }
};

template <>
struct DataType<foxglove_bridge::GenericServiceType> {
  static const char* value(const foxglove_bridge::GenericServiceType& m) {
    return m.type.c_str();
  }

  static const char* value() {
    return "*";
  }
};
}  // namespace ros::service_traits

namespace ros::serialization {

template <>
struct Serializer<foxglove_bridge::GenericServiceType> {
  template <typename Stream>
  inline static void write(Stream& stream, const foxglove_bridge::GenericServiceType& m) {
    m.write(stream);
  }

  template <typename Stream>
  inline static void read(Stream& stream, foxglove_bridge::GenericServiceType& m) {
    m.read(stream);
  }

  inline static uint32_t serializedLength(const foxglove_bridge::GenericServiceType& m) {
    return m.data.size();
  }
};
}  // namespace ros::serialization
