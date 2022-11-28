#include <chrono>
#include <future>
#include <thread>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <websocketpp/config/asio_client.hpp>

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

  std_msgs::String rosMsg;
  rosMsg.data = "hello world";
  pub.publish(rosMsg);

  const auto clientCount = 3;
  for (auto i = 0; i < clientCount; ++i) {
    (void)i;

    // Set up text message handler to resolve the promise when the string topic is advertised
    foxglove::Client<websocketpp::config::asio_client> wsClient;
    std::promise<nlohmann::json> channelPromise;
    auto channelFuture = channelPromise.get_future();
    wsClient.setTextMessageHandler([&topic_name, &channelPromise](const std::string& payload) {
      const auto msg = nlohmann::json::parse(payload);
      const auto& op = msg.at("op").get<std::string>();
      if (op == "advertise") {
        for (const auto& channel : msg.at("channels")) {
          if (topic_name == channel.at("topic")) {
            channelPromise.set_value(channel);
          }
        }
      }
    });

    // Connect the client and wait for the channel future
    ASSERT_EQ(std::future_status::ready, wsClient.connect(URI).wait_for(std::chrono::seconds(5)));
    ASSERT_EQ(std::future_status::ready, channelFuture.wait_for(std::chrono::seconds(5)));

    // Set up binary message handler to resolve when a binary message has been received
    std::promise<std::pair<const uint8_t*, size_t>> msgPromise;
    auto msgFuture = msgPromise.get_future();
    wsClient.setBinaryMessageHandler([&msgPromise](const uint8_t* data, size_t dataLength) {
      const size_t offset = 1 + 4 + 8;
      msgPromise.set_value({data + offset, dataLength - offset});
    });

    // Subscribe to the channel that corresponds to the string topic
    const auto channelId = channelFuture.get().at("id").get<foxglove::ChannelId>();
    wsClient.subscribe({{1, channelId}});

    // Wait until we have received a binary message and test that it is the right one
    ASSERT_EQ(std::future_status::ready, msgFuture.wait_for(std::chrono::seconds(5)));
    const auto& [data, dataLength] = msgFuture.get();
    ASSERT_EQ(sizeof(HELLO_WORLD_BINARY), dataLength);
    EXPECT_TRUE(std::memcmp(HELLO_WORLD_BINARY, data, dataLength));
  }
}

TEST(SmokeTest, testPublishing) {
  foxglove::Client<websocketpp::config::asio_client> wsClient;

  foxglove::ClientAdvertisement advertisement;
  advertisement.channelId = 1;
  advertisement.topic = "/foo";
  advertisement.encoding = "ros1";
  advertisement.schemaName = "std_msgs/String";

  ros::NodeHandle nh;
  std::promise<std::string> msgPromise;
  auto msgFuture = msgPromise.get_future();
  auto subscriber = nh.subscribe<std_msgs::String>(
    advertisement.topic, 10, [&msgPromise](const std_msgs::String::ConstPtr& msg) {
      ROS_INFO_STREAM("DATA: " << msg->data);
      msgPromise.set_value(msg->data);
    });

  ASSERT_EQ(std::future_status::ready, wsClient.connect(URI).wait_for(std::chrono::seconds(5)));

  wsClient.advertise({advertisement});
  std::this_thread::sleep_for(std::chrono::seconds(1));
  wsClient.publish(advertisement.channelId, HELLO_WORLD_BINARY, sizeof(HELLO_WORLD_BINARY));
  wsClient.unadvertise({advertisement.channelId});

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
