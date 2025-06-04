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
      // std::cout << "Minimum Angle (in degrees)" << (float)msg->angle_min*180/(float)M_PI << std::endl;
      // std::cout << "Maximum Angle (in degrees)" << (float)msg->angle_max*180/(float)M_PI << std::endl;
      size_t num_obstacles = (size_t)msg->ranges.size();

      // std::cout << "No. of Angles Stored: " << (int) n_obstacles << std::endl;
      // std::cout << "Length of ranges[]: " <<  << std::endl;
      float* r_obstacles = (float*) malloc(sizeof(float) * num_obstacles);
      for(int i = 0; i < (int)num_obstacles; i++){
        r_obstacles[i] = (float)msg->ranges[i];
        // std::cout << r_obstacles[i] << std::endl;
      }
      // ---Obstacle Set Identification---
      float* valuefunc2D = (float*) malloc(sizeof(float) * Grid::getSizeX() * Grid::getSizeY());
      // rt_reachability::computeObstacleSet(r_obstacles,(int)num_obstacles,(float)msg->angle_min,(float)msg->angle_max,(float)msg->angle_increment,valuefunc2D);
      
      // ---Primitive Grid (Obstacle-Set Only) Visualization in Terminal---

      // std::cout << std::endl;
      // for(int i = 0;i < Grid::getSizeX();i++){
      //   for(int j = 0;j < Grid::getSizeY();j++){
      //     if(valuefunc2D[i*Grid::getSizeY() + j] == -1.0) std::cout << "- ";
      //     else if(valuefunc2D[i*Grid::getSizeY() + j] == -2.0) std::cout << "X ";
      //     else std::cout << "O ";
      //     // std::cout << valuefunc2D[i*Grid::getSizeY() + j] << " ";
      //   }
      //   std::cout << std::endl;
      // }
      // std::cout << std::endl;

      // ---Cylindrical-To-Cartesian---
      // float* x_obstacles = (float*) malloc(sizeof(float) * num_obstacles);
      // float* y_obstacles = (float*) malloc(sizeof(float) * num_obstacles);
      // rt_reachability::cylToCart(r_obstacles,(int)num_obstacles,(float)msg->angle_min,(float)msg->angle_max,(float)msg->angle_increment,x_obstacles,y_obstacles);
      // for(int i = 0;i < (int)num_obstacles;i++){
      //   float exp = r_obstacles[i];
      //   float calc = sqrt(x_obstacles[i]*x_obstacles[i] + y_obstacles[i]*y_obstacles[i]);
      //   std::cout << "Cartesian Coordinates of the Obstacles: (" << x_obstacles[i] << "," << y_obstacles[i] << "); Expected distance: " << exp << "; Calculated distance: " <<  calc << " ; Error: " << exp - calc << std::endl;
      // }

      // ---SDF Computation---
      rt_reachability::computeSDF(r_obstacles,num_obstacles,(float)msg->angle_min,(float)msg->angle_max,(float)msg->angle_increment,valuefunc2D);
      std::cout << std::endl;
      for(int i = 0;i < Grid::getSizeX();i++){
        for(int j = 0;j < Grid::getSizeY();j++){
          std::cout << valuefunc2D[i*Grid::getSizeY() + j] << " ";
        }
        std::cout << std::endl;
      }
      std::cout << std::endl;
      this->count++;
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  };

  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SDFNode>();

    Grid::setSize(50,50,50,50);
    Grid::setLowerBounds(0.0,-2.0,0.1,-((float)M_PI)/6);
    Grid::setUpperBounds(4.0,+2.0,5.0,+((float)M_PI)/6);
    
    // The first cudaMalloc command takes almost a tenth of a second to execute
    // firstInitMem() takes the brunt of the slow cudaMalloc command as it allocates and frees memory to a dummy pointer
    firstInitMem();
    // rclcpp::spin(node);
    while(rclcpp::ok() && node->count < 1){
      rclcpp::spin_some(node);
    }
    if(rclcpp::ok())
      rclcpp::shutdown();
    return 0;
}
