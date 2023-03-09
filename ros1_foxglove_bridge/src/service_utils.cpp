#include <ros/connection.h>
#include <ros/service_manager.h>
#include <ros/service_server_link.h>

#include <foxglove_bridge/service_utils.hpp>

namespace foxglove_bridge {

std::future<std::string> retrieveServiceType(const std::string& serviceName) {
  auto link = ros::ServiceManager::instance()->createServiceServerLink(serviceName, false, "*", "*",
                                                                       {{"probe", "1"}});
  auto promise = std::make_shared<std::promise<std::string>>();
  auto future = promise->get_future();

  link->getConnection()->setHeaderReceivedCallback(
    [promise = std::move(promise)](const ros::ConnectionPtr&, const ros::Header& header) mutable {
      std::string serviceType;
      if (header.getValue("type", serviceType)) {
        promise->set_value(serviceType);
      } else {
        promise->set_exception(std::make_exception_ptr(
          std::runtime_error("Key 'type' not found in service connection header")));
      }
      return true;
    });

  return future;
}

}  // namespace foxglove_bridge
