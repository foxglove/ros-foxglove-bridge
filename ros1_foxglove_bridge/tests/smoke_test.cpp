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

class ParameterTest : public ::testing::Test {
public:
  using PARAM_1_TYPE = std::string;
  inline static const std::string PARAM_1_NAME = "/node_1/string_param";
  inline static const PARAM_1_TYPE PARAM_1_DEFAULT_VALUE = "hello";

  using PARAM_2_TYPE = std::vector<double>;
  inline static const std::string PARAM_2_NAME = "/node_2/int_array_param";
  inline static const PARAM_2_TYPE PARAM_2_DEFAULT_VALUE = {1.2, 2.1, 3.3};

protected:
  void SetUp() override {
    _nh = ros::NodeHandle();
    _nh.setParam(PARAM_1_NAME, PARAM_1_DEFAULT_VALUE);
    _nh.setParam(PARAM_2_NAME, PARAM_2_DEFAULT_VALUE);

    _wsClient = std::make_shared<foxglove::Client<websocketpp::config::asio_client>>();
    ASSERT_EQ(std::future_status::ready, _wsClient->connect(URI).wait_for(std::chrono::seconds(5)));
  }

  ros::NodeHandle _nh;
  std::shared_ptr<foxglove::Client<websocketpp::config::asio_client>> _wsClient;
};

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

TEST_F(ParameterTest, testGetAllParams) {
  const std::string requestId = "req-testGetAllParams";
  _wsClient->getParameters({}, requestId);
  std::vector<foxglove::Parameter> params;
  ASSERT_NO_THROW(params = foxglove::waitForParameters(_wsClient, requestId));
  EXPECT_GE(params.size(), 2UL);
}

TEST_F(ParameterTest, testGetNonExistingParameters) {
  const std::string requestId = "req-testGetNonExistingParameters";
  _wsClient->getParameters(
    {"/foo_1/non_existing_parameter", "/foo_2/non_existing/nested_parameter"}, requestId);
  std::vector<foxglove::Parameter> params;
  ASSERT_NO_THROW(params = foxglove::waitForParameters(_wsClient, requestId));
  EXPECT_TRUE(params.empty());
}

TEST_F(ParameterTest, testGetParameters) {
  const std::string requestId = "req-testGetParameters";
  _wsClient->getParameters({PARAM_1_NAME, PARAM_2_NAME}, requestId);
  std::vector<foxglove::Parameter> params;
  ASSERT_NO_THROW(params = foxglove::waitForParameters(_wsClient, requestId));
  EXPECT_EQ(2UL, params.size());
  auto p1Iter = std::find_if(params.begin(), params.end(), [](const auto& param) {
    return param.getName() == PARAM_1_NAME;
  });
  auto p2Iter = std::find_if(params.begin(), params.end(), [](const auto& param) {
    return param.getName() == PARAM_2_NAME;
  });
  ASSERT_NE(p1Iter, params.end());
  EXPECT_EQ(PARAM_1_DEFAULT_VALUE, p1Iter->getValue<PARAM_1_TYPE>());
  ASSERT_NE(p2Iter, params.end());
  EXPECT_EQ(PARAM_2_DEFAULT_VALUE, p2Iter->getValue<PARAM_2_TYPE>());
}

TEST_F(ParameterTest, testSetParameters) {
  const PARAM_1_TYPE newP1value = "world";
  const PARAM_2_TYPE newP2value = {4.1, 5.5, 6.6};

  const std::vector<foxglove::Parameter> parameters = {
    foxglove::Parameter(PARAM_1_NAME, newP1value),
    foxglove::Parameter(PARAM_2_NAME, newP2value),
  };

  _wsClient->setParameters(parameters);
  const std::string requestId = "req-testSetParameters";
  _wsClient->getParameters({PARAM_1_NAME, PARAM_2_NAME}, requestId);
  std::vector<foxglove::Parameter> params;
  ASSERT_NO_THROW(params = foxglove::waitForParameters(_wsClient, requestId));
  EXPECT_EQ(2UL, params.size());
  auto p1Iter = std::find_if(params.begin(), params.end(), [](const auto& param) {
    return param.getName() == PARAM_1_NAME;
  });
  auto p2Iter = std::find_if(params.begin(), params.end(), [](const auto& param) {
    return param.getName() == PARAM_2_NAME;
  });
  ASSERT_NE(p1Iter, params.end());
  EXPECT_EQ(newP1value, p1Iter->getValue<PARAM_1_TYPE>());
  ASSERT_NE(p2Iter, params.end());
  EXPECT_EQ(newP2value, p2Iter->getValue<PARAM_2_TYPE>());
}

TEST_F(ParameterTest, testParameterSubscription) {
  _wsClient->subscribeParameterUpdates({PARAM_1_NAME});
  _wsClient->setParameters({foxglove::Parameter(PARAM_1_NAME, "foo")});
  std::vector<foxglove::Parameter> params;
  ASSERT_NO_THROW(params = foxglove::waitForParameters(_wsClient, "", std::chrono::seconds(5)));
  ASSERT_EQ(1UL, params.size());
  EXPECT_EQ(params.front().getName(), PARAM_1_NAME);

  _wsClient->unsubscribeParameterUpdates({PARAM_1_NAME});
  _wsClient->setParameters({foxglove::Parameter(PARAM_1_NAME, "bar")});
  EXPECT_THROW((void)foxglove::waitForParameters(_wsClient, "", std::chrono::seconds(5)),
               std::runtime_error);
}

TEST_F(ParameterTest, testGetParametersParallel) {
  // Connect a few clients (in parallel) and make sure that they all receive parameters
  auto clients = {
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
    std::make_shared<foxglove::Client<websocketpp::config::asio_client>>(),
  };

  std::vector<std::future<std::vector<foxglove::Parameter>>> futures;
  for (auto client : clients) {
    futures.push_back(
      std::async(std::launch::async, [client]() -> std::vector<foxglove::Parameter> {
        if (std::future_status::ready == client->connect(URI).wait_for(std::chrono::seconds(5))) {
          const std::string requestId = "req-123";
          client->getParameters({}, requestId);
          const auto parameters =
            foxglove::waitForParameters(client, requestId, std::chrono::seconds(5));
          return parameters;
        }
        return {};
      }));
  }

  for (auto& future : futures) {
    ASSERT_EQ(std::future_status::ready, future.wait_for(std::chrono::seconds(5)));
    std::vector<foxglove::Parameter> parameters;
    EXPECT_NO_THROW(parameters = future.get());
    EXPECT_GE(parameters.size(), 2UL);
  }
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
