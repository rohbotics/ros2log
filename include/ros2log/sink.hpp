#ifndef SINK_H
#define SINK_H

enum class Log_Levels;
struct MetaData;

using SinkFunction = std::function<void(Log_Levels, MetaData, const char *)>;
struct Sink {
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

#endif