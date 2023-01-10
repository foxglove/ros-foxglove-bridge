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

class ParameterTest : public ::testing::Test {
public:
  using PARAM_1_TYPE = std::string;
  inline static const std::string NODE_1_NAME = "node_1";
  inline static const std::string PARAM_1_NAME = "string_param";
  inline static const PARAM_1_TYPE PARAM_1_DEFAULT_VALUE = "hello";

  using PARAM_2_TYPE = std::vector<int64_t>;
  inline static const std::string NODE_2_NAME = "node_2";
  inline static const std::string PARAM_2_NAME = "int_array_param";
  inline static const PARAM_2_TYPE PARAM_2_DEFAULT_VALUE = {1, 2, 3};

protected:
  void SetUp() override {
    _paramNode1 = rclcpp::Node::make_shared(NODE_1_NAME);
    auto p1Param = rcl_interfaces::msg::ParameterDescriptor{};
    p1Param.name = PARAM_1_NAME;
    p1Param.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    p1Param.read_only = false;
    _paramNode1->declare_parameter(p1Param.name, PARAM_1_DEFAULT_VALUE, p1Param);

    _paramNode2 = rclcpp::Node::make_shared(NODE_2_NAME);
    auto p2Param = rcl_interfaces::msg::ParameterDescriptor{};
    p2Param.name = PARAM_2_NAME;
    p2Param.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
    p2Param.read_only = false;
    _paramNode2->declare_parameter(p2Param.name, PARAM_2_DEFAULT_VALUE, p2Param);

    _executor.add_node(_paramNode1);
    _executor.add_node(_paramNode2);
    _executorThread = std::thread([this]() {
      _executor.spin();
    });

    _wsClient = std::make_shared<foxglove::Client<websocketpp::config::asio_client>>();
    ASSERT_EQ(std::future_status::ready, _wsClient->connect(URI).wait_for(std::chrono::seconds(5)));
  }

  void TearDown() override {
    _executor.cancel();
    _executorThread.join();
  }

  rclcpp::executors::SingleThreadedExecutor _executor;
  rclcpp::Node::SharedPtr _paramNode1;
  rclcpp::Node::SharedPtr _paramNode2;
  std::thread _executorThread;
  std::shared_ptr<foxglove::Client<websocketpp::config::asio_client>> _wsClient;
};

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
    {"/foo_1.non_existing_parameter", "/foo_2.non_existing.nested_parameter"}, requestId);
  std::vector<foxglove::Parameter> params;
  ASSERT_NO_THROW(params = foxglove::waitForParameters(_wsClient, requestId));
  EXPECT_TRUE(params.empty());
}

TEST_F(ParameterTest, testGetParameters) {
  const auto p1 = NODE_1_NAME + "." + PARAM_1_NAME;
  const auto p2 = NODE_2_NAME + "." + PARAM_2_NAME;

  const std::string requestId = "req-testGetParameters";
  _wsClient->getParameters({p1, p2}, requestId);
  std::vector<foxglove::Parameter> params;
  ASSERT_NO_THROW(params = foxglove::waitForParameters(_wsClient, requestId));
  EXPECT_EQ(2UL, params.size());
  auto p1Iter = std::find_if(params.begin(), params.end(), [&p1](const auto& param) {
    return param.getName() == p1;
  });
  auto p2Iter = std::find_if(params.begin(), params.end(), [&p2](const auto& param) {
    return param.getName() == p2;
  });
  ASSERT_NE(p1Iter, params.end());
  EXPECT_EQ(PARAM_1_DEFAULT_VALUE, p1Iter->getValue<PARAM_1_TYPE>());
  ASSERT_NE(p2Iter, params.end());
  EXPECT_EQ(PARAM_2_DEFAULT_VALUE, p2Iter->getValue<PARAM_2_TYPE>());
}

TEST_F(ParameterTest, testSetParameters) {
  const auto p1 = NODE_1_NAME + "." + PARAM_1_NAME;
  const auto p2 = NODE_2_NAME + "." + PARAM_2_NAME;
  const PARAM_1_TYPE newP1value = "world";
  const PARAM_2_TYPE newP2value = {4, 5, 6};

  const std::vector<foxglove::Parameter> parameters = {
    foxglove::Parameter(p1, newP1value),
    foxglove::Parameter(p2, newP2value),
  };

  _wsClient->setParameters(parameters);
  const std::string requestId = "req-testSetParameters";
  _wsClient->getParameters({p1, p2}, requestId);
  std::vector<foxglove::Parameter> params;
  ASSERT_NO_THROW(params = foxglove::waitForParameters(_wsClient, requestId));
  EXPECT_EQ(2UL, params.size());
  auto p1Iter = std::find_if(params.begin(), params.end(), [&p1](const auto& param) {
    return param.getName() == p1;
  });
  auto p2Iter = std::find_if(params.begin(), params.end(), [&p2](const auto& param) {
    return param.getName() == p2;
  });
  ASSERT_NE(p1Iter, params.end());
  EXPECT_EQ(newP1value, p1Iter->getValue<PARAM_1_TYPE>());
  ASSERT_NE(p2Iter, params.end());
  EXPECT_EQ(newP2value, p2Iter->getValue<PARAM_2_TYPE>());
}

TEST_F(ParameterTest, testSetParametersWithReqId) {
  const auto p1 = NODE_1_NAME + "." + PARAM_1_NAME;
  const PARAM_1_TYPE newP1value = "world";
  const std::vector<foxglove::Parameter> parameters = {
    foxglove::Parameter(p1, newP1value),
  };

  const std::string requestId = "req-testSetParameters";
  _wsClient->setParameters(parameters, requestId);
  std::vector<foxglove::Parameter> params;
  ASSERT_NO_THROW(params = foxglove::waitForParameters(_wsClient, requestId));
  EXPECT_EQ(1UL, params.size());
}

TEST_F(ParameterTest, testParameterSubscription) {
  const auto p1 = NODE_1_NAME + "." + PARAM_1_NAME;

  _wsClient->subscribeParameterUpdates({p1});
  _wsClient->setParameters({foxglove::Parameter(p1, "foo")});
  std::vector<foxglove::Parameter> params;
  ASSERT_NO_THROW(params = foxglove::waitForParameters(_wsClient));
  ASSERT_EQ(1UL, params.size());
  EXPECT_EQ(params.front().getName(), p1);

  _wsClient->unsubscribeParameterUpdates({p1});
  _wsClient->setParameters({foxglove::Parameter(p1, "bar")});
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
