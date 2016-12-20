#include <ros/ros.h>
#include <fmt/format.h>

#include <string>
#include <utility>

template <typename... Args>
void log_warn(const char *f, Args &&... args){
	fmt::MemoryWriter w;
	w.write(f, std::forward<Args>(args)...);
	ROS_WARN("%s", w.c_str());
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "fmt_test");

	int foo = 42;
	log_warn("The meaning of life {}", foo);
	log_warn("This time in hex: {:x}", foo);
	log_warn("Binary: {:b}", foo);

	double bar = 3.14159265;
	log_warn("Pi {}", bar);

	std::string hi = "hola";
	log_warn("{}", hi);
}