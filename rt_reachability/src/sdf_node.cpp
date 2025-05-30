#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class SDFNode : public rclcpp::Node {

  public:
    int count;
    SDFNode() : Node("sdf_node") {
      this->count = 0;
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", default_qos,
          std::bind(&SDFNode::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      // Count: 0
      // Number of angles (array size): 1080
      // Number of angles (estimated): 1080
      std::cout << "Count: " << this->count << std::endl;
      std::cout << "Number of angles (array size): " << msg->ranges.size() << std::endl;
      std::cout << "Number of angles (estimated): " << (msg->angle_max - msg->angle_min)/(msg->angle_increment) << std::endl;
      this->count++;
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  };

  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SDFNode>();
    RCLCPP_INFO(node->get_logger(), "SDF is being computed");
    while(rclcpp::ok() && node->count < 1){
      rclcpp::spin_some(node);
    }
    if(rclcpp::ok())
      rclcpp::shutdown();
    return 0;
}
