#define ASIO_STANDALONE

#include <chrono>
#include <future>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp_components/component_manager.hpp>
#include <std_msgs/msg/string.hpp>
#include <websocketpp/config/asio_client.hpp>

#include <foxglove_bridge/test/test_client.hpp>
#include <foxglove_bridge/websocket_client.hpp>

constexpr char URI[] = "ws://localhost:8765";

// Binary representation of std_msgs/msg/String for "hello world"
constexpr uint8_t HELLO_WORLD_BINARY[] = {0,   1,   0,   0,  12,  0,   0,   0,   104, 101,
                                          108, 108, 111, 32, 119, 111, 114, 108, 100, 0};

TEST(SmokeTest, testConnection) {
  foxglove::Client<websocketpp::config::asio_client> wsClient;
  EXPECT_EQ(std::future_status::ready, wsClient.connect(URI).wait_for(std::chrono::seconds(5)));
}

TEST(SmokeTest, testSubscription) {
  // Publish a string message on a latched ros topic
  const std::string topic_name = "/pub_topic";
  std_msgs::msg::String rosMsg;
  rosMsg.data = "hello world";

  auto node = rclcpp::Node::make_shared("tester");
  rclcpp::QoS qos = rclcpp::QoS{rclcpp::KeepLast(1lu)};
  qos.transient_local();
  auto pub = node->create_publisher<std_msgs::msg::String>(topic_name, qos);
  pub->publish(rosMsg);

  // Connect a few clients and make sure that they receive the correct message
  const auto clientCount = 3;
  for (auto i = 0; i < clientCount; ++i) {
    std::vector<uint8_t> msgData;
    ASSERT_NO_THROW(msgData = foxglove::connectClientAndReceiveMsg(URI, topic_name));
    ASSERT_EQ(sizeof(HELLO_WORLD_BINARY), msgData.size());
    EXPECT_EQ(0, std::memcmp(HELLO_WORLD_BINARY, msgData.data(), msgData.size()));
  }
}

TEST(SmokeTest, testSubscriptionParallel) {
  // Publish a string message on a latched ros topic
  const std::string topic_name = "/pub_topic";
  std_msgs::msg::String rosMsg;
  rosMsg.data = "hello world";

  auto node = rclcpp::Node::make_shared("tester");
  rclcpp::QoS qos = rclcpp::QoS{rclcpp::KeepLast(1lu)};
  qos.transient_local();
  auto pub = node->create_publisher<std_msgs::msg::String>(topic_name, qos);
  pub->publish(rosMsg);

  // Connect a few clients (in parallel) and make sure that they receive the correct message
  std::vector<std::future<std::vector<uint8_t>>> futures;
  const auto clientCount = 3;
  for (auto i = 0; i < clientCount; ++i) {
    futures.push_back(
      std::async(std::launch::async, foxglove::connectClientAndReceiveMsg, URI, topic_name));
  }

  for (auto& future : futures) {
    ASSERT_EQ(std::future_status::ready, future.wait_for(std::chrono::seconds(5)));
    auto msgData = future.get();
    ASSERT_EQ(sizeof(HELLO_WORLD_BINARY), msgData.size());
    EXPECT_EQ(0, std::memcmp(HELLO_WORLD_BINARY, msgData.data(), msgData.size()));
  }
}

TEST(SmokeTest, testPublishing) {
  foxglove::ClientAdvertisement advertisement;
  advertisement.channelId = 1;
  advertisement.topic = "/foo";
  advertisement.encoding = "cdr";
  advertisement.schemaName = "std_msgs/String";

  // Set up a ROS node with a subscriber
  std::promise<std::string> msgPromise;
  auto msgFuture = msgPromise.get_future();
  auto node = rclcpp::Node::make_shared("tester");
  auto sub = node->create_subscription<std_msgs::msg::String>(
    advertisement.topic, 10, [&msgPromise](const std_msgs::msg::String::SharedPtr msg) {
      msgPromise.set_value(msg->data);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Set up the client, advertise and publish the binary message
  foxglove::Client<websocketpp::config::asio_client> wsClient;
  ASSERT_EQ(std::future_status::ready, wsClient.connect(URI).wait_for(std::chrono::seconds(5)));
  wsClient.advertise({advertisement});
  std::this_thread::sleep_for(std::chrono::seconds(1));
  wsClient.publish(advertisement.channelId, HELLO_WORLD_BINARY, sizeof(HELLO_WORLD_BINARY));
  wsClient.unadvertise({advertisement.channelId});

  // Ensure that we have received the correct message via our ROS subscriber
  const auto ret = executor.spin_until_future_complete(msgFuture, std::chrono::seconds(1));
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, ret);
  EXPECT_EQ("hello world", msgFuture.get());
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  const size_t numThreads = 2;
  auto executor =
    rclcpp::executors::MultiThreadedExecutor::make_shared(rclcpp::ExecutorOptions{}, numThreads);

  rclcpp_components::ComponentManager componentManager(executor, "ComponentManager");
  const auto componentResources = componentManager.get_component_resources("foxglove_bridge");

  if (componentResources.empty()) {
    RCLCPP_INFO(componentManager.get_logger(), "No loadable resources found");
    return EXIT_FAILURE;
  }

  auto componentFactory = componentManager.create_component_factory(componentResources.front());
  auto node = componentFactory->create_node_instance(rclcpp::NodeOptions());
  executor->add_node(node.get_node_base_interface());

  std::thread spinnerThread([&executor]() {
    executor->spin();
  });

  const auto testResult = RUN_ALL_TESTS();
  executor->cancel();
  spinnerThread.join();
  rclcpp::shutdown();

  return testResult;
}
