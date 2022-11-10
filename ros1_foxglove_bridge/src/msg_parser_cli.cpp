#include <iostream>

#include <foxglove_bridge/msg_parser.hpp>

int main(int argc, char const* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <msg_type>" << std::endl;
    return EXIT_FAILURE;
  }

  foxglove_bridge::MsgParser msg_parser;

  try {
    std::cout << msg_parser.get_message_schema(argv[1]) << std::endl;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
