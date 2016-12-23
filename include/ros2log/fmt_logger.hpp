#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2log/logger.hpp>

#include <chrono>

class FmtLogger : public Logger {
 public:
  using Logger::Logger;

  template <typename... Args>
  void debug(const char *f, Args &&... args) const {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto data = fmt::format(f, std::forward<Args>(args)...);
    output(Log_Levels::DEBUG,
           add_metadata("DEBUG", seconds.count(), nano_seconds.count(), data)
               .c_str());
  }

  template <typename... Args>
  void info(const char *f, Args &&... args) const {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto data = fmt::format(f, std::forward<Args>(args)...);
    output(Log_Levels::INFO,
           add_metadata("INFO", seconds.count(), nano_seconds.count(), data)
               .c_str());
  }

  template <typename... Args>
  void warn(const char *f, Args &&... args) const {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto data = fmt::format(f, std::forward<Args>(args)...);
    output(Log_Levels::WARN,
           add_metadata("WARN", seconds.count(), nano_seconds.count(), data)
               .c_str());
  }

  template <typename... Args>
  void error(const char *f, Args &&... args) const {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto data = fmt::format(f, std::forward<Args>(args)...);
    output(Log_Levels::ERROR,
           add_metadata("ERROR", seconds.count(), nano_seconds.count(), data)
               .c_str());
  }

  template <typename... Args>
  void fatal(const char *f, Args &&... args) const {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto data = fmt::format(f, std::forward<Args>(args)...);
    output(Log_Levels::FATAL,
           add_metadata("FATAL", seconds.count(), nano_seconds.count(), data)
               .c_str());
  }

 protected:
  std::string add_metadata(const std::string &level, int secs, int nsecs,
                           const std::string &data) const {
    return fmt::format("{level:<5}: [{secs}.{nsecs}] {data}",
                       FMT_CAPTURE(level, secs, nsecs, data));
  }
};