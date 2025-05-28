#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "math.h"

using std::placeholders::_1;

class SDFNode : public rclcpp::Node {

public:
    float max_x = 6.0;
    float max_y = 3.0;
    SDFNode() : Node("sdf_node") {

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", default_qos,
        std::bind(&SDFNode::topic_callback, this, _1));
    }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {

  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SDFNode>();
  RCLCPP_INFO(node->get_logger(), "SDF is being computed");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
