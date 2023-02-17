#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace foxglove_bridge {

class CallbackQueue {
public:
  CallbackQueue(size_t numThreads = 1)
      : _quit(false) {
    for (size_t i = 0; i < numThreads; ++i) {
      _workerThreads.push_back(std::thread(&CallbackQueue::doWork, this));
    }
  }

  ~CallbackQueue() {
    stop();
  }

  void stop() {
    _quit = true;
    _cv.notify_all();
    for (auto& thread : _workerThreads) {
      thread.join();
    }
  }

  void addCallback(std::function<void(void)> cb) {
    if (_quit) {
      return;
    }
    std::unique_lock<std::mutex> lock(_mutex);
    _callbackQueue.push_back(cb);
    _cv.notify_one();
  }

private:
  void doWork() {
    while (!_quit) {
      std::unique_lock<std::mutex> lock(_mutex);
      _cv.wait(lock, [this] {
        return (_quit || !_callbackQueue.empty());
      });
      if (_quit) {
        break;
      } else if (!_callbackQueue.empty()) {
        std::function<void(void)> cb = _callbackQueue.front();
        _callbackQueue.pop_front();
        lock.unlock();
        cb();
      }
    }
  }

  std::atomic<bool> _quit;
  std::mutex _mutex;
  std::condition_variable _cv;
  std::deque<std::function<void(void)>> _callbackQueue;
  std::vector<std::thread> _workerThreads;
};

}  // namespace foxglove_bridge
