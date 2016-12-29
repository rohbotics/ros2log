#ifndef SINK_H
#define SINK_H

enum class Log_Levels;
struct LogMessage;

using SinkFunction = std::function<void(LogMessage)>;
struct Sink {
  Sink(std::string sink_name, Log_Levels starting_level, SinkFunction function)
      : name(sink_name),
        enabled(true),
        output_level(starting_level),
        output_function(function) {}
  Sink() = default;
  ~Sink() = default;

  std::string name;
  bool enabled;
  Log_Levels output_level;
  SinkFunction output_function = nullptr;
};

#endif