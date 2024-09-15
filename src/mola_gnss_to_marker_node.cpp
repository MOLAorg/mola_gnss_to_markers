#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MolaGnssToMarkerNode : public rclcpp::Node {
public:
  MolaGnssToMarkerNode() : Node("mola_gnss_to_marker_node") {
    RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s",
                "mola_gnss_to_marker_node");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MolaGnssToMarkerNode>());
  rclcpp::shutdown();
  return 0;
}