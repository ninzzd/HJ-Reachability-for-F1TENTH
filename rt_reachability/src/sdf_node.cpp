#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rt_reachability/Grid.hpp"
#include "rt_reachability/SDF.hpp"
#include <cmath>

using std::placeholders::_1;
using namespace rt_reachability;
class SDFNode : public rclcpp::Node {
  public:
    int count;
    SDFNode() : Node("sdf") {
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
      size_t n_obstacles = (size_t)(msg->angle_max - msg->angle_min/msg->angle_increment);
      float* r_obstacles = (float*) malloc(sizeof(float) * n_obstacles);
      for(int i = 0; i < (int)n_obstacles; i++){
        r_obstacles[i] = (float)msg->ranges[i];
        std::cout << r_obstacles[i] << std::endl;
      }
      float* valuefunc2D = (float*) malloc(sizeof(float) * Grid::getSizeX() * Grid::getSizeY());
      computeObstacleSet(r_obstacles,(int)n_obstacles,msg->angle_min, msg->angle_max,msg->angle_increment,valuefunc2D);
      std::cout << "Successfully computed the obstacle set." << std::endl;
      for(int i = 0;i < Grid::getSizeX();i++){
        for(int j = 0;j < Grid::getSizeY();j++){
          if(valuefunc2D[i*Grid::getSizeY() + j] == -1.0) std::cout << "- ";
          else std::cout << "O ";
        }
        std::cout << std::endl;
      }
      // Commented the below code to first test the obstacle set computation using CUDA
      // DO NOT DELETE THE BELOW COMMENTED CODE
      // float* x_obstacles = (float*) malloc(sizeof(float) * n_obstacles);
      // float* y_obstacles = (float*) malloc(sizeof(float) * n_obstacles);
      // int j = 0;
      // for(int i = 0;i < msg->ranges.size();i++){ // This loop can be GPU-Parallelized
      //   float r = msg->ranges[i];
      //   float theta = msg->angle_min + i*msg->angle_increment;
      //   float x = r*std::cos(theta);
      //   float y = r*std::sin(theta);
      //   if((x <= Grid::getMaxX() && x >= Grid::getMinX()) && (y <= Grid::getMaxY() && y >= Grid::getMinY()) ){
      //     x_obstacles[j] = x;
      //     y_obstacles[j] = y;
      //   }
      //   else{
      //     x_obstacles[j] = (float)MAXFLOAT;
      //     y_obstacles[j] = (float)MAXFLOAT;
      //   }
      // }
      this->count++;
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  };

  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SDFNode>();
    RCLCPP_INFO(node->get_logger(), "SDF is being computed");

    Grid::setSize(50,50,50,50);
    Grid::setLowerBounds(0.0,-2.0,0.1,-((float)M_PI)/6);
    Grid::setUpperBounds(4.0,+2.0,5.0,+((float)M_PI)/6);
    
    // rclcpp:spin(node);
    while(rclcpp::ok() && node->count < 1){
      rclcpp::spin_some(node);
    }
    if(rclcpp::ok())
      rclcpp::shutdown();
    return 0;
}
