#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2log/logger.hpp>

#include <chrono>

class FmtLogger {
 public:
  FmtLogger(std::shared_ptr<rclcpp::node::Node> node) : logger(node){};

  ~FmtLogger() = default;

  template <typename... Args>
  void debug(const char *f, Args &&... args) const {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto data = fmt::format(f, std::forward<Args>(args)...);
    logger.debug(
        add_metadata("DEBUG", seconds.count(), nano_seconds.count(), data));
  }

  template <typename... Args>
  void info(const char *f, Args &&... args) const {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto data = fmt::format(f, std::forward<Args>(args)...);
    logger.info(
        add_metadata("INFO", seconds.count(), nano_seconds.count(), data));
  }

  template <typename... Args>
  void warn(const char *f, Args &&... args) const {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto data = fmt::format(f, std::forward<Args>(args)...);
    logger.warn(
        add_metadata("WARN", seconds.count(), nano_seconds.count(), data));
  }

  template <typename... Args>
  void error(const char *f, Args &&... args) const {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto data = fmt::format(f, std::forward<Args>(args)...);
    logger.error(
        add_metadata("ERROR", seconds.count(), nano_seconds.count(), data));
  }

  template <typename... Args>
  void fatal(const char *f, Args &&... args) const {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto data = fmt::format(f, std::forward<Args>(args)...);
    logger.fatal(
        add_metadata("FATAL", seconds.count(), nano_seconds.count(), data));
  }

 private:
  Logger logger;

  std::string add_metadata(const std::string &level, int secs, int nsecs,
                           const std::string &data) const {
    return fmt::format("{level}: [{secs}.{nsecs}] {data}",
                       FMT_CAPTURE(level, secs, nsecs, data));
  }
};