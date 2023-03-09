#include <ros/connection.h>
#include <ros/service_manager.h>
#include <ros/service_server_link.h>

#include <foxglove_bridge/service_utils.hpp>

namespace foxglove_bridge {

std::shared_future<std::string> retrieveServiceType(const std::string& serviceName) {
  auto link = ros::ServiceManager::instance()->createServiceServerLink(serviceName, false, "*", "*",
                                                                       {{"probe", "1"}});
  auto promise = std::make_shared<std::promise<std::string>>();
  std::shared_future<std::string> future(promise->get_future());

  link->getConnection()->setHeaderReceivedCallback(
    [promise](const ros::ConnectionPtr&, const ros::Header& header) {
      std::string serviceType;
      if (header.getValue("type", serviceType)) {
        promise->set_value(serviceType);
      } else {
        promise->set_exception(
          std::make_exception_ptr(std::runtime_error("Failed to retrieve service type")));
      }
      return true;
    });

  return future;
}

}  // namespace foxglove_bridge
