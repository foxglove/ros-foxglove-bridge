#define ASIO_STANDALONE

#include <chrono>
#include <future>
#include <thread>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/builtin_string.h>
#include <websocketpp/config/asio_client.hpp>

#include <foxglove_bridge/test/test_client.hpp>
#include <foxglove_bridge/websocket_client.hpp>

constexpr char URI[] = "ws://localhost:9876";

// Binary representation of std_msgs/String for "hello world"
constexpr uint8_t HELLO_WORLD_BINARY[] = {11,  0,  0,   0,   104, 101, 108, 108,
                                          111, 32, 119, 111, 114, 108, 100};

TEST(SmokeTest, testConnection) {
  foxglove::Client<websocketpp::config::asio_client> wsClient;
  EXPECT_EQ(std::future_status::ready, wsClient.connect(URI).wait_for(std::chrono::seconds(5)));
}

TEST(SmokeTest, testSubscription) {
  // Publish a string message on a latched ros topic
  const std::string topic_name = "/pub_topic";
  ros::NodeHandle nh;
  auto pub = nh.advertise<std_msgs::String>(topic_name, 10, true);
  pub.publish(std::string("hello world"));

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
  ros::NodeHandle nh;
  auto pub = nh.advertise<std_msgs::String>(topic_name, 10, true);
  pub.publish(std::string("hello world"));

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
  foxglove::Client<websocketpp::config::asio_client> wsClient;

  foxglove::ClientAdvertisement advertisement;
  advertisement.channelId = 1;
  advertisement.topic = "/foo";
  advertisement.encoding = "ros1";
  advertisement.schemaName = "std_msgs/String";

  // Set up a ROS node with a subscriber
  ros::NodeHandle nh;
  std::promise<std::string> msgPromise;
  auto msgFuture = msgPromise.get_future();
  auto subscriber = nh.subscribe<std_msgs::String>(
    advertisement.topic, 10, [&msgPromise](const std_msgs::String::ConstPtr& msg) {
      msgPromise.set_value(msg->data);
    });

  // Set up the client, advertise and publish the binary message
  ASSERT_EQ(std::future_status::ready, wsClient.connect(URI).wait_for(std::chrono::seconds(5)));
  wsClient.advertise({advertisement});
  std::this_thread::sleep_for(std::chrono::seconds(1));
  wsClient.publish(advertisement.channelId, HELLO_WORLD_BINARY, sizeof(HELLO_WORLD_BINARY));
  wsClient.unadvertise({advertisement.channelId});

  // Ensure that we have received the correct message via our ROS subscriber
  const auto msgResult = msgFuture.wait_for(std::chrono::seconds(1));
  ASSERT_EQ(std::future_status::ready, msgResult);
  EXPECT_EQ("hello world", msgFuture.get());
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  // Give the server some time to start
  std::this_thread::sleep_for(std::chrono::seconds(2));

  ros::AsyncSpinner spinner(1);
  spinner.start();
  const auto testResult = RUN_ALL_TESTS();
  spinner.stop();

  return testResult;
}
