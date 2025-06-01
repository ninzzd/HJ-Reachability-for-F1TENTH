#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rt_reachability/Grid.hpp"
#include <cmath>

using std::placeholders::_1;

class SDFNode : public rclcpp::Node {
  public:
    // int count;
    SDFNode() : Node("sdf_node") {
      // this->count = 0;
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
      size_t n_obstacles = (size_t)(msg->angle_max - msg->angle_min/msg->angle_increment);
      float* x_obstacles = (float*) malloc(sizeof(float) * n_obstacles);
      float* y_obstacles = (float*) malloc(sizeof(float) * n_obstacles);
      float* r_obstacles = (float*) malloc(sizeof(float) * n_obstacles);
      int j = 0;
      for(int i = 0;i < msg->ranges.size();i++){ // This loop can be GPU-Parallelized
        float r = msg->ranges[i];
        float theta = msg->angle_min + i*msg->angle_increment;
        float x = r*std::cos(theta);
        float y = r*std::sin(theta);
        if((x <= rt_reachability::Grid::getMaxX() && x >= rt_reachability::Grid::getMinX()) && (y <= rt_reachability::Grid::getMaxY() && y >= rt_reachability::Grid::getMinY()) ){
          x_obstacles[j] = x;
          y_obstacles[j] = y;
        }
        else{
          x_obstacles[j] = (float)MAXFLOAT;
          y_obstacles[j] = (float)MAXFLOAT;
        }
      }
      
      // this->count++;
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  };

  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SDFNode>();
    RCLCPP_INFO(node->get_logger(), "SDF is being computed");

    rt_reachability::Grid::setSize(50,50,50,50);
    rt_reachability::Grid::setLowerBounds(0.0,-2.0,0.1,-((float)M_PI)/6);
    rt_reachability::Grid::setUpperBounds(4.0,+2.0,5.0,-((float)M_PI)/6);
    
    rclcpp:spin(node);
    // while(rclcpp:ok() && node->count < 1){
    //   rclcpp::spin_some(node);
    // }
    // if(rclcpp::ok())

      rclcpp::shutdown();
    return 0;
}
