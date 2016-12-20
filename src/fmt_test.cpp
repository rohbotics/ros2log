#include <ros/ros.h>
#include <fmt/format.h>

#include <string>
#include <utility>


// log_warn uses rvalue refrences and perfect forwarding
template <typename... Args>
void log_warn(const char *f, Args &&... args){
	fmt::MemoryWriter w;
	w.write(f, std::forward<Args>(args)...);
	ROS_WARN("%s", w.c_str());
}

// log_error uses const lvalue refs
// It also uses std::string just to demo
template <typename... Args>
void log_error(const char *f, const Args &... args){
	std::string s = fmt::format(f, args...);
	ROS_ERROR("%s", s.c_str());
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "fmt_test");

	int foo = 42;
	log_warn("The meaning of life {}", foo);
	log_warn("This time in hex: {:x}", foo);
	log_warn("Binary: {:b}", foo);

	double bar = 3.14159265;
	log_error("Pi {}", bar);

	std::string hi = "hola";
	log_error("{}", hi);
}
