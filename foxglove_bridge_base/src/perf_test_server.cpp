#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <unordered_map>

#include <boost/program_options.hpp>
#include <nlohmann/json.hpp>

#include <foxglove_bridge/server_factory.hpp>

namespace po = boost::program_options;
using ConnectionHandle = websocketpp::connection_hdl;

struct ChannelConfig {
  std::string topic;
  int sizeInBytes;
  double frequency;
};

std::atomic<bool> doAbort = false;
std::unordered_map<foxglove::ChannelId, std::vector<ConnectionHandle>> subscriptions;
std::shared_mutex subscriptionMutex;
const std::vector<ChannelConfig> DEFAULT_CHANNELS = {
  {"10 bytes @ 100 Hz", 10, 100.0},  {"10 bytes @ 500 Hz", 10, 500.0},
  {"10 KB @ 100 Hz", 10000, 100.0},  {"10 KB @ 500 Hz", 10000, 500.0},
  {"100 KB @ 10 Hz", 100000, 10.0},  {"100 KB @ 50 Hz", 100000, 50.0},
  {"1 MB @ 10 Hz", 1000000, 10.0},   {"1 MB @ 50 Hz", 1000000, 50.0},
  {"5 MB @ 10 Hz", 5000000, 10.0},   {"5 MB @ 20 Hz", 5000000, 20.0},
  {"10 MB @ 10 Hz", 10000000, 10.0}, {"10 MB @ 20 Hz", 10000000, 20.0}};

void from_json(const nlohmann::json& j, ChannelConfig& p) {
  p.topic = j["topic"].get<std::string>();
  p.frequency = j["frequency"].get<double>();
  p.sizeInBytes = j["sizeInBytes"].get<int>();
}

void signal_handler(int) {
  std::cout << "Received SIGINT, exiting..." << std::endl;
  doAbort = true;
}

void logCallback(foxglove::WebSocketLogLevel level, char const* msg) {
  switch (level) {
    case foxglove::WebSocketLogLevel::Debug:
      std::cout << "[DEBUG]" << msg << std::endl;
      break;
    case foxglove::WebSocketLogLevel::Info:
      std::cout << "[INFO]" << msg << std::endl;
      break;
    case foxglove::WebSocketLogLevel::Warn:
      std::cout << "[WARN]" << msg << std::endl;
      break;
    case foxglove::WebSocketLogLevel::Error:
      std::cerr << "[ERROR]" << msg << std::endl;
      break;
    case foxglove::WebSocketLogLevel::Critical:
      std::cerr << "[FATAL]" << msg << std::endl;
      break;
  }
}

void runFakeChannel(foxglove::ServerInterface<ConnectionHandle>* server,
                    foxglove::ChannelId channelId, int sizeInBytes, double frequency) {
  const std::vector<uint8_t> data(sizeInBytes);
  if (frequency <= 0.0) return;
  const uint64_t periodMs = static_cast<uint64_t>(1000.0 / frequency);

  std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();
  while (!doAbort && server) {
    // Calculate the time to sleep to maintain the desired frequency
    const std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
    const uint64_t elapsedMs =
      std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();
    const uint64_t sleepTimeMs = periodMs - elapsedMs;

    if (sleepTimeMs > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepTimeMs));
    }

    // Update the lastTime for the next iteration
    lastTime = std::chrono::steady_clock::now();

    std::shared_lock<std::shared_mutex> lock(subscriptionMutex);
    const auto& subscriberHandles = subscriptions[channelId];
    for (const auto hdl : subscriberHandles) {
      server->sendMessage(hdl, channelId, 0, data.data(), data.size());
    }
  }
}

int main(int argc, char** argv) {
  std::string address;
  int port;
  std::string channelConfigFile;
  int sendBufferLimit;
  std::string certfile;
  std::string keyfile;

  po::options_description desc("Options");
  desc.add_options()("address", po::value<std::string>(&address)->default_value("0.0.0.0"),
                     "Address to bind to")("port", po::value<int>(&port)->default_value(8765),
                                           "Port to listen to")(
    "send-buffer-limit", po::value<int>(&sendBufferLimit)->default_value(2e7),
    "Send buffer limit in bytes")("channel-config", po::value<std::string>(&channelConfigFile),
                                  "Path to JSON file with custom channels in the format "
                                  "{topic: string, frequency: number, sizeInBytes:number}[].")(
    "compression", "Enable per-message deflate compression")(
    "certfile", po::value<std::string>(&certfile), "Path to the certificate to use for TLS")(
    "keyfile", po::value<std::string>(&keyfile), "Path to the private key to use for TLS")(
    "help", "Produce help message");

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const po::error& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << "Usage: " << argv[0] << " [options]" << std::endl << desc << std::endl;
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << "Usage: " << argv[0] << " [options]" << std::endl << desc << std::endl;
    return EXIT_SUCCESS;
  }

  const std::vector<ChannelConfig> channelConfig =
    channelConfigFile.empty()
      ? DEFAULT_CHANNELS
      : nlohmann::json::parse(std::ifstream(channelConfigFile)).get<std::vector<ChannelConfig>>();

  foxglove::ServerOptions serverOptions;
  serverOptions.capabilities = {};
  serverOptions.sendBufferLimitBytes = sendBufferLimit;
  serverOptions.sessionId = std::to_string(std::time(nullptr));
  serverOptions.useCompression = vm.count("compression");
  serverOptions.useTls = !certfile.empty() && !keyfile.empty();
  serverOptions.certfile = certfile;
  serverOptions.keyfile = keyfile;
  serverOptions.clientTopicWhitelistPatterns = {std::regex(".*")};

  const auto logHandler = std::bind(&logCallback, std::placeholders::_1, std::placeholders::_2);
  auto server = foxglove::ServerFactory::createServer<ConnectionHandle>("perf-test-server",
                                                                        logHandler, serverOptions);

  std::vector<foxglove::ChannelWithoutId> channelsWithoutId;
  for (const auto& channelConfig : channelConfig) {
    if (channelConfig.sizeInBytes > sendBufferLimit) {
      std::cerr << "Message size of channel '" << channelConfig.topic
                << "' is larger than the send buffer limit. No messages will be sent "
                   "to clients."
                << std::endl;
    }
    channelsWithoutId.emplace_back(foxglove::ChannelWithoutId{channelConfig.topic, "", "", "", ""});
  }
  const auto channelIds = server->addChannels(channelsWithoutId);
  if (channelIds.size() != channelConfig.size()) {
    std::cerr << "Failed to add channels" << std::endl;
    return EXIT_FAILURE;
  }

  std::vector<std::thread> channelThreads;
  for (size_t i = 0; i < channelConfig.size(); i++) {
    const auto& cfg = channelConfig[i];
    const auto channelId = channelIds[i];
    channelThreads.emplace_back(runFakeChannel, server.get(), channelId, cfg.sizeInBytes,
                                cfg.frequency);
  }

  foxglove::ServerHandlers<ConnectionHandle> hdlrs;
  hdlrs.subscribeHandler = [&](foxglove::ChannelId channelId, ConnectionHandle hdl) {
    std::unique_lock<std::shared_mutex> lock(subscriptionMutex);
    auto& subscriberHandles = subscriptions[channelId];
    subscriberHandles.push_back(hdl);
  };
  hdlrs.unsubscribeHandler = [&](foxglove::ChannelId channelId, ConnectionHandle hdl) {
    std::unique_lock<std::shared_mutex> lock(subscriptionMutex);
    auto& subscriberHandles = subscriptions[channelId];
    const auto it = std::remove_if(subscriberHandles.begin(), subscriberHandles.end(),
                                   [hdl](ConnectionHandle other) {
                                     return !hdl.owner_before(other) && !other.owner_before(hdl);
                                   });
    subscriberHandles.erase(it, subscriberHandles.end());
  };
  server->setHandlers(std::move(hdlrs));
  server->start(address, port);
  std::signal(SIGINT, signal_handler);

  while (!doAbort) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  for (auto& thread : channelThreads) {
    thread.join();
  }
  server->stop();
  return EXIT_SUCCESS;
}
