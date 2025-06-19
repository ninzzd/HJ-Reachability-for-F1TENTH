#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rt_reachability/Grid.hpp"
#include "rt_reachability/SDF.hpp"
#include <cmath>
#include "math.h"

#define min(x,y) ((((x)<(y))?(x):(y)))
#define max(x,y) ((((x)>(y))?(x):(y)))
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
        auto default_qos = rclcpp::SensorDataQoS().keep_last(1);

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", default_qos,
            std::bind(&SDFNode::topic_callback, this, _1));
        publisher_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/sdf_grid_pointcloud",10
        );
        publisher_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/reachability_grid_pointcloud",10
        );
    }

  private:
    void visualizeObstacleSet(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        size_t num_obstacles = (size_t)msg->ranges.size();
        float* r_obstacles = (float*) malloc(sizeof(float) * num_obstacles);
        float* valuefunc2D = (float*) malloc(sizeof(float) * Grid::getSizeX() * Grid::getSizeY());
        for(int i = 0; i < (int)num_obstacles; i++){
            r_obstacles[i] = (float)msg->ranges[i];
        }
        // ---Obstacle Set Identification---
        rt_reachability::computeObstacleSet(r_obstacles,(int)num_obstacles,(float)msg->angle_min,(float)msg->angle_max,(float)msg->angle_increment,valuefunc2D);
        
        // ---Primitive Grid (Obstacle-Set Only) Visualization in Terminal---

        std::cout << std::endl;
        for(int i = 0;i < Grid::getSizeX();i++){
            for(int j = 0;j < Grid::getSizeY();j++){
            if(valuefunc2D[i*Grid::getSizeY() + j] == -1.0) std::cout << "- ";
            else if(valuefunc2D[i*Grid::getSizeY() + j] == -2.0) std::cout << "X ";
            else std::cout << "O ";
            // std::cout << valuefunc2D[i*Grid::getSizeY() + j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    void visualizeSDF(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        size_t num_obstacles = (size_t)msg->ranges.size();
        float* r_obstacles = (float*) malloc(sizeof(float) * num_obstacles);
        float* valuefunc2D = (float*) malloc(sizeof(float) * Grid::getSizeX() * Grid::getSizeY());
        for(int i = 0; i < (int)num_obstacles; i++){
            r_obstacles[i] = (float)msg->ranges[i];
        }

        // --- SDF Computation ---
        rt_reachability::computeSDF(r_obstacles,num_obstacles,(float)msg->angle_min,(float)msg->angle_max,(float)msg->angle_increment,valuefunc2D);

        // --- Marker Visualization --- (Commented as RViz hangs a lot with Marker msgs, not suitable)
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
        pc_msg.header.frame_id = "ego_racecar/laser_model";
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
        publisher_1->publish(pc_msg);
        free(valuefunc2D);
    }
    void visualizeReachabilitySetSlice(const sensor_msgs::msg::LaserScan::SharedPtr msg,float v, float theta, int num){
        size_t num_obstacles = (size_t)msg->ranges.size();
        float* r_obstacles = (float*) malloc(sizeof(float) * num_obstacles);
        for(int i = 0; i < (int)num_obstacles; i++){
            r_obstacles[i] = (float)msg->ranges[i];
        }
        Grid::initializeGrid(r_obstacles,num_obstacles,(float)msg->angle_min,(float)msg->angle_max,(float)msg->angle_increment);
        Grid::Point* grid = Grid::computeReachability(num);
        float* grid_slice = (float*) malloc(sizeof(float)*Grid::getSizeX()*Grid::getSizeY());
        int k = (int)floorf((v - Grid::getMinV())*(Grid::getSizeV() - 1)/(Grid::getMaxV() - Grid::getMinV()));
        k = max(0,min(k,Grid::getSizeV()-1));
        int l = (int)floorf((v - Grid::getMinTheta())*(Grid::getSizeTheta() - 1)/(Grid::getMaxTheta() - Grid::getMinTheta()));
        l = max(0,min(k,Grid::getSizeTheta()-1));
        for(int i = 0;i < Grid::getSizeX();i++){
            for(int j = 0;j < Grid::getSizeY();j++){
                grid_slice[i*Grid::getSizeY() + j] = grid[l*Grid::getSizeX()*Grid::getSizeY()*Grid::getSizeV() + k*Grid::getSizeX()*Grid::getSizeY() + i*Grid::getSizeY() + j].value;
            }
        }
        free(grid);

        sensor_msgs::msg::PointCloud2 pc_msg;
        pc_msg.header.stamp = rclcpp::Time(0);
        pc_msg.header.frame_id = "ego_racecar/laser_model";
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
            float z = grid_slice[i*Grid::getSizeY() + j];
            memcpy(&pc_msg.data[byte_idx+0],&x,sizeof(float));
            memcpy(&pc_msg.data[byte_idx+4],&y,sizeof(float));
            memcpy(&pc_msg.data[byte_idx+8],&z,sizeof(float));
        }
        }
        publisher_2->publish(pc_msg);
        free(grid_slice);
    }
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::cout << "Callback function for LiDAR has started..." << std::endl;
        // visualizeSDF(msg);
        visualizeReachabilitySetSlice(msg,3.0f,0.0f,75);
        // The following line is not being returned
        std::cout << "Callback function for LiDAR has ended..." << std::endl;
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_1;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_2;
  };

  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SDFNode>();

    Grid::setSize(50,50,50,50);
    Grid::setLowerBounds(0.0,-1.0,0.1,-((float)M_PI)/2);
    Grid::setUpperBounds(2.0,+1.0,5.0,+((float)M_PI)/2);
    Grid::setCarLength(0.33);
    Grid::setInputParams(10,10,0.01,10.0,-((float)M_PI)/6,+((float)M_PI)/6);
    Grid::computeDeltaT();
    // firstInitMem() takes the brunt of the slow cudaMalloc command as it allocates and frees memory to a dummy pointer
    firstInitMem();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // Callbacks run one at a time
    rclcpp::shutdown();
    return 0;
    return 0;
}
