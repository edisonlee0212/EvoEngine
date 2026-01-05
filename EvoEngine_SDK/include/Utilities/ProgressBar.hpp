#pragma once
#include <algorithm>
#include <iostream>
#include <mutex>
#include <string>

namespace evo_engine {
class ProgressBar {
 public:
  enum class Display { Percentage, Absolute };

  ProgressBar(size_t start, size_t end, std::string label, Display mode = Display::Percentage, size_t width = 50,
              char fill = '=', char head = '>', char empty = ' ')
      : start_(start),
        end_(end),
        mode_(mode),
        width_(width),
        fill_(fill),
        head_(head),
        empty_(empty),
        label_(std::move(label)) {
    if (end_ <= start_)
      end_ = start_ + 1;  // avoid division by zero
    bar_.resize(width_, empty_);
    std::cout << label_ << ":" << std::endl;
  }

  void Update(size_t value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value = std::clamp(value, start_, end_);
    double progress = double(value - start_) / double(end_ - start_);

    size_t filled = static_cast<size_t>(progress * width_);

    // Update only what changed
    if (filled != filled_) {
      if (filled > filled_) {
        for (size_t i = filled_; i < std::min(filled, width_); ++i)
          bar_[i] = fill_;
      } else {
        for (size_t i = filled_; i > filled; --i)
          bar_[i - 1] = empty_;
      }
      filled_ = filled;
    }

    // Update head position
    if (filled_ < width_)
      bar_[filled_] = head_;

    Render(value, progress);
  }

  void Finish() {
    Update(end_);
    std::cout << std::endl;
  }

 private:
  void Render(size_t value, double progress) {
    std::cout << "\r[" << bar_ << "] ";

    switch (mode_) {
      case Display::Percentage:
        std::cout << static_cast<int>(progress * 100) << "%";
        break;

      case Display::Absolute:
        std::cout << value << "/" << end_;
        break;
    }

    std::cout << std::flush;
  }

  size_t start_;
  size_t end_;
  Display mode_;
  std::mutex mutex_;

  size_t width_;
  char fill_;
  char head_;
  char empty_;

  std::string label_;
  std::string bar_;
  size_t filled_ = 0;
};
}  // namespace evo_engine
