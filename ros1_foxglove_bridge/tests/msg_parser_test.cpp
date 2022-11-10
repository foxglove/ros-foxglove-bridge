#include <gtest/gtest.h>

#include <foxglove_bridge/msg_parser.hpp>

constexpr char GENDEPS_POSE_MSG[] =
  "# A representation of pose in free space, composed of position and orientation. \n"
  "Point position\n"
  "Quaternion orientation\n"
  "\n"
  "================================================================================\n"
  "MSG: geometry_msgs/Point\n"
  "# This contains the position of a point in free space\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n"
  "\n"
  "================================================================================\n"
  "MSG: geometry_msgs/Quaternion\n"
  "# This represents an orientation in free space in quaternion form.\n"
  "\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n"
  "float64 w\n";

TEST(message_parser, msg_not_found) {
  foxglove_bridge::MsgParser msg_parser;
  EXPECT_THROW((void)msg_parser.get_message_schema("foo/bar"),
               foxglove_bridge::MsgNotFoundException);
  EXPECT_TRUE(true);
}

TEST(message_parser, invalid_msg_type) {
  foxglove_bridge::MsgParser msg_parser;
  EXPECT_THROW((void)msg_parser.get_message_schema("foo/bar/baz"),
               foxglove_bridge::InvalidMessageType);
  EXPECT_TRUE(true);
}

TEST(message_parser, msg_schema) {
  foxglove_bridge::MsgParser msg_parser;
  EXPECT_EQ(GENDEPS_POSE_MSG, msg_parser.get_message_schema("geometry_msgs/Pose"));
  EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
