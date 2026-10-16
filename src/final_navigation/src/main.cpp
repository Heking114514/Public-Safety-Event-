#include "visual_navigation/waypoint_navigator.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace {

bool HasExplicitNodeRemap(int argc, char **argv) {
  for (int index = 0; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument.rfind("__node:=", 0) == 0 ||
        argument.rfind("__node:", 0) == 0) {
      return true;
    }
    if ((argument == "-r" || argument == "--remap") && index + 1 < argc) {
      const std::string remap(argv[index + 1]);
      if (remap.rfind("__node:=", 0) == 0 ||
          remap.rfind("__node:", 0) == 0) {
        return true;
      }
    }
  }
  return false;
}

std::vector<std::string> WithContrastNodeName(int argc, char **argv) {
  if (HasExplicitNodeRemap(argc, argv)) {
    return std::vector<std::string>(argv, argv + argc);
  }

  std::vector<std::string> arguments;
  arguments.reserve(static_cast<std::size_t>(argc) + 3U);
  bool remap_added = false;
  for (int index = 0; index < argc; ++index) {
    arguments.emplace_back(argv[index]);
    if (!remap_added && arguments.back() == "--ros-args") {
      arguments.emplace_back("--remap");
      arguments.emplace_back("__node:=final_navigation");
      remap_added = true;
    }
  }
  if (!remap_added) {
    arguments.emplace_back("--ros-args");
    arguments.emplace_back("--remap");
    arguments.emplace_back("__node:=final_navigation");
  }
  return arguments;
}

} // namespace

int main(int argc, char **argv) {
  auto arguments = WithContrastNodeName(argc, argv);
  std::vector<char *> raw_arguments;
  raw_arguments.reserve(arguments.size());
  for (auto &argument : arguments)
    raw_arguments.push_back(&argument[0]);

  rclcpp::init(static_cast<int>(raw_arguments.size()), raw_arguments.data());
  rclcpp::spin(std::make_shared<WaypointNavigator>());
  rclcpp::shutdown();
  return 0;
}
