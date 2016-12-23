#pragma once
#include <ros2log/enums.hpp>

using SinkFunction = std::function<void(Log_Levels, const char *)>;
class Sink {
 public:
  Sink(std::string sink_name, Log_Levels starting_level, SinkFunction function)
      : name(sink_name),
        output_level(starting_level),
        output_function(function) {}
  Sink() = default;
  ~Sink() = default;

  std::string name;
  Log_Levels output_level;
  SinkFunction output_function = nullptr;
};