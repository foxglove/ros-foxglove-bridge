#include <gtest/gtest.h>

#include <foxglove_bridge/json_to_ros.hpp>

TEST(JsonToRosTest, EmptyStringMsg) {
  const std::string payload = "{}";

  auto babelFish = ros2_babel_fish::BabelFish::make_shared();

  ros2_babel_fish::CompoundMessage::SharedPtr output;
  auto res = foxglove_bridge::jsonMessageToRos(payload, "std_msgs/msg/String", babelFish, output);
  ASSERT_FALSE(res.has_value()) << "Error converting JSON to ROS: " << res.value().what();
  ASSERT_TRUE(output) << "Output message is null";
  EXPECT_EQ(output->datatype(), "std_msgs::msg::String");
  EXPECT_EQ(output->memberCount(), 1);
  EXPECT_EQ((*output)["data"].type(), ros2_babel_fish::MessageType::String);
  EXPECT_EQ((*output)["data"].value<std::string>(), "");
  EXPECT_TRUE(output->type_erased_message());
}

TEST(JsonToRosTest, StringMsg) {
  const std::string payload = R"(
    { "data": "Hello, World!" }
  )";

  auto babelFish = ros2_babel_fish::BabelFish::make_shared();

  ros2_babel_fish::CompoundMessage::SharedPtr output;
  auto res = foxglove_bridge::jsonMessageToRos(payload, "std_msgs/msg/String", babelFish, output);
  ASSERT_FALSE(res.has_value()) << "Error converting JSON to ROS: " << res.value().what();
  ASSERT_TRUE(output) << "Output message is null";
  EXPECT_EQ(output->datatype(), "std_msgs::msg::String");
  EXPECT_EQ(output->memberCount(), 1);
  EXPECT_EQ((*output)["data"].type(), ros2_babel_fish::MessageType::String);
  EXPECT_EQ((*output)["data"].value<std::string>(), "Hello, World!");
  EXPECT_TRUE(output->type_erased_message());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
