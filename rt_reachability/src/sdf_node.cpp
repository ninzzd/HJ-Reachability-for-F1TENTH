#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rt_reachability/Grid.hpp"
#include "rt_reachability/SDF.hpp"
#include <cmath>

using std::placeholders::_1;
using namespace rt_reachability;
class SDFNode : public rclcpp::Node {
  public:
    int count;
    int sent;
    visualization_msgs::msg::Marker prev_marker;
    SDFNode() : Node("sdf") {
      this->count = 0;
      this->sent = 0;
      auto default_qos = rclcpp::SensorDataQoS();

      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", default_qos,
          std::bind(&SDFNode::topic_callback, this, _1));
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/sdf_grid_pointcloud",10
      );
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
      float* valuefunc2D = (float*) malloc(sizeof(float) * Grid::getSizeX() * Grid::getSizeY());
      for(int i = 0; i < (int)num_obstacles; i++){
        r_obstacles[i] = (float)msg->ranges[i];
        // std::cout << r_obstacles[i] << std::endl;
      }
      // ---Obstacle Set Identification---
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

      // --- Cylindrical-To-Cartesian ---
      // float* x_obstacles = (float*) malloc(sizeof(float) * num_obstacles);
      // float* y_obstacles = (float*) malloc(sizeof(float) * num_obstacles);

      // --- Terminal Display of Cartesian Coordinates and Computed Distance ---
      // rt_reachability::cylToCart(r_obstacles,(int)num_obstacles,(float)msg->angle_min,(float)msg->angle_max,(float)msg->angle_increment,x_obstacles,y_obstacles);
      // for(int i = 0;i < (int)num_obstacles;i++){
      //   float exp = r_obstacles[i];
      //   float calc = sqrt(x_obstacles[i]*x_obstacles[i] + y_obstacles[i]*y_obstacles[i]);
      //   std::cout << "Cartesian Coordinates of the Obstacles: (" << x_obstacles[i] << "," << y_obstacles[i] << "); Expected distance: " << exp << "; Calculated distance: " <<  calc << " ; Error: " << exp - calc << std::endl;
      // }

      // --- SDF Computation ---
      rt_reachability::computeSDF(r_obstacles,num_obstacles,(float)msg->angle_min,(float)msg->angle_max,(float)msg->angle_increment,valuefunc2D);

      // --- Terminal Visualization of SDF ---

      // std::cout << std::endl;
      // for(int i = 0;i < Grid::getSizeX();i++){
      //   for(int j = 0;j < Grid::getSizeY();j++){
      //     std::cout << valuefunc2D[i*Grid::getSizeY() + j] << " ";
      //   }
      //   std::cout << std::endl;
      // }
      // std::cout << std::endl;

      // --- Marker Visualization of SDF in RViz ---
      // if(this->sent == 1){
      //   prev_marker.action = visualization_msgs::msg::Marker::DELETE;
      //   publisher_->publish(prev_marker);
      // }

      // --- Marker Visualization ---

      // visualization_msgs::msg::Marker marker = visualization_msgs::msg::Marker();
      // marker.header.stamp = rclcpp::Time(0);
      // marker.header.frame_id = "ego_racecar/base_link";
      // marker.id = 1;
      // marker.ns = "SDF Grid";
      // marker.type = visualization_msgs::msg::Marker::POINTS;
      // marker.action = visualization_msgs::msg::Marker::ADD;
      // marker.scale.x = 0.05;
      // marker.scale.y = 0.05;
      // marker.scale.z = 0.05;
      // marker.color.r = 1.0f;
      // marker.color.g = 0.0f;
      // marker.color.b = 0.0f;
      // marker.color.a = 1.0f;
      // for(int i = 0;i < Grid::getSizeX();i++){
      //   for(int j = 0;j < Grid::getSizeY();j++){
      //     geometry_msgs::msg::Point point;
      //     point.x = Grid::getMaxX() - i*(float)(Grid::getMaxX() - Grid::getMinX())/(Grid::getSizeX()-1);
      //     point.y = Grid::getMaxY() - j*(float)(Grid::getMaxY() - Grid::getMinY())/(Grid::getSizeY()-1);
      //     point.z = valuefunc2D[i*Grid::getSizeY() + j];
      //     marker.points.push_back(point);
      //   }
      // }
      // marker.lifetime = rclcpp::Duration::from_seconds(1);
      // publisher_->publish(marker);
      // prev_marker = marker;
      // if(this->sent == 0 ) this->sent = 1;

      // --- PointCloud Visualization ---

      sensor_msgs::msg::PointCloud2 pc_msg;
      pc_msg.header.stamp = rclcpp::Time(0);
      pc_msg.header.frame_id = "ego_racecar/base_link";
      pc_msg.height = 1;
      free(r_obstacles);
      pc_msg.width = Grid::getSizeX()*Grid::getSizeY();
      pc_msg.is_dense = true;
      sensor_msgs::msg::PointField field;
      field.name = "x"; field.offset = 0; field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
      pc_msg.fields.push_back(field);
      field.name = "y"; field.offset = 4; pc_msg.fields.push_back(field);
      field.name = "z"; field.offset = 8; pc_msg.fields.push_back(field);
      pc_msg.point_step = 12; // 3 * 4 bytes (float32)
      pc_msg.row_step = pc_msg.point_step * pc_msg.width;
      pc_msg.data.resize(pc_msg.row_step*pc_msg.height);
      for(int i = 0;i < Grid::getSizeX();i++){
        for(int j = 0;j < Grid::getSizeY();j++){
          size_t point_idx = i*Grid::getSizeY() + j;
          size_t byte_idx = point_idx *pc_msg.point_step;
          float x = Grid::getMaxX() - i*(Grid::getMaxX() - Grid::getMinX())/(Grid::getSizeX() - 1);
          float y = Grid::getMaxY() - j*(Grid::getMaxY() - Grid::getMinY())/(Grid::getSizeY() - 1);
          float z = valuefunc2D[i*Grid::getSizeY() + j];
          memcpy(&pc_msg.data[byte_idx+0],&x,sizeof(float));
          memcpy(&pc_msg.data[byte_idx+4],&y,sizeof(float));
          memcpy(&pc_msg.data[byte_idx+8],&z,sizeof(float));
        }
      }
      publisher_->publish(pc_msg);

      free(valuefunc2D);
      // this->count++;
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  };

  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SDFNode>();

    Grid::setSize(50,50,50,50);
    Grid::setLowerBounds(0.0,-2.0,0.1,-((float)M_PI)/6);
    Grid::setUpperBounds(4.0,+2.0,5.0,+((float)M_PI)/6);
    
    // Bufferring the bash display to allows in-place display
    std::cout << std::endl;
    std::cout << std::endl;
    // The first cudaMalloc command takes almost a tenth of a second to execute
    // firstInitMem() takes the brunt of the slow cudaMalloc command as it allocates and frees memory to a dummy pointer
    firstInitMem();
    rclcpp::spin(node);
    // while(rclcpp::ok() && node->count < 1){
    //   rclcpp::spin_some(node);
    // }
    // if(rclcpp::ok())
    rclcpp::shutdown();
    return 0;
}
