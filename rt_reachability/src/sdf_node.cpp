#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rt_reachability/Grid.hpp"
#include "rt_reachability/SDF.hpp"
#include <mutex>
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
    // visualization_msgs::msg::Marker prev_marker;
    SDFNode() : Node("sdf") {
        this->count = 0;
        this->sent = 0;
        this->declare_parameter<int>("num_iter",10);
        auto default_qos = rclcpp::SensorDataQoS().keep_last(1);
        Grid::setSize(50,50,50,50);
        Grid::setLowerBounds(0.0,-1.0,-5.0,-((float)M_PI)/2);
        Grid::setUpperBounds(2.0,+1.0,5.0,+((float)M_PI)/2);
        Grid::setCarLength(0.33);
        Grid::setInputParams(50,20,-5.0f,5.0f,-((float)M_PI)/6,+((float)M_PI)/6);
        Grid::setValueFunctionBounds(-2.5f,+5.0f);
        Grid::setHorizon(0.1);
        Grid::computeDeltaT();
        firstInitMem();
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", default_qos,
            std::bind(&SDFNode::laser_callback, this, _1)
        );
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
             std::bind(&SDFNode::odom_callback, this, _1)
        );
        sdf_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/sdf_grid_pointcloud", 10
        );
        reach_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/reachability_grid_pointcloud",    10
        );
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",   10
        );
        odom_new = false;
        laser_new = false;
        rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
        timer = this->create_wall_timer(std::chrono::milliseconds(4),std::bind(&SDFNode::timer_callback,this));
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
        sdf_pub->publish(pc_msg);
        free(valuefunc2D);
    }
    void visualizeReachabilitySetSlice(Grid::Point* grid, float v, float theta){
        // size_t num_obstacles = (size_t)msg->ranges.size();
        // float* r_obstacles = (float*) malloc(sizeof(float) * num_obstacles);
        // for(int i = 0; i < (int)num_obstacles; i++){
        //     r_obstacles[i] = (float)msg->ranges[i];
        // }
        // Grid::initializeGrid(r_obstacles,num_obstacles,(float)msg->angle_min,(float)msg->angle_max,(float)msg->angle_increment);
        // grid = Grid::computeReachability();
        float* grid_slice = (float*) malloc(sizeof(float)*Grid::getSizeX()*Grid::getSizeY());
        int k = (int)floorf((v - Grid::getMinV())*(Grid::getSizeV() - 1)/(Grid::getMaxV() - Grid::getMinV()));
        k = max(0,min(k,Grid::getSizeV()-1));
        int l = (int)floorf((theta - Grid::getMinTheta())*(Grid::getSizeTheta() - 1)/(Grid::getMaxTheta() - Grid::getMinTheta()));
        l = max(0,min(l,Grid::getSizeTheta()-1));
        for(int i = 0;i < Grid::getSizeX();i++){
            for(int j = 0;j < Grid::getSizeY();j++){
                grid_slice[i*Grid::getSizeY() + j] = grid[l*Grid::getSizeX()*Grid::getSizeY()*Grid::getSizeV() + k*Grid::getSizeX()*Grid::getSizeY() + i*Grid::getSizeY() + j].value;
            }
        }

        sensor_msgs::msg::PointCloud2 pc_msg;
        pc_msg.header.stamp = rclcpp::Time(0);
        pc_msg.header.frame_id = "ego_racecar/laser_model";
        pc_msg.height = 1;
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
        reach_pub->publish(pc_msg);
        free(grid_slice);
    }
    Grid::Point* computeBRT(const sensor_msgs::msg::LaserScan msg){
        size_t num_obstacles = (size_t)msg.ranges.size();
        float* r_obstacles = (float*) malloc(sizeof(float) * num_obstacles);
        for(int i = 0; i < (int)num_obstacles; i++){
            r_obstacles[i] = (float)msg.ranges[i];
        }
        Grid::initializeGrid(r_obstacles,num_obstacles,(float)msg.angle_min,(float)msg.angle_max,(float)msg.angle_increment);
        free(r_obstacles);
        return Grid::computeReachability();
    }
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        laser_mutex.lock();
        current_laser = *msg;
        laser_new = true;
        laser_mutex.unlock();
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        odom_mtx.lock();
        current_odom = *msg;
        odom_new = true;
        odom_mtx.unlock();
    }
    void timer_callback(){
        // odom_lock = true;
        // laser_lock = true;
        nav_msgs::msg::Odometry local_odom;
        sensor_msgs::msg::LaserScan local_laser;

        float x0,y0,theta0;
        double roll, pitch, yaw;
        bool toExit = false;
        odom_mtx.lock();
            if(odom_new == true){
                local_odom = current_odom;
                odom_new = false;
            }
            else toExit = true;
        odom_mtx.unlock();
        if(toExit == true) return;
        x0 = local_odom.pose.pose.position.x;
        y0 = local_odom.pose.pose.position.y;
        tf2::Quaternion q0(local_odom.pose.pose.orientation.w,local_odom.pose.pose.orientation.x,local_odom.pose.pose.orientation.y,local_odom.pose.pose.orientation.z);
        tf2::Matrix3x3(q0).getRPY(roll,pitch,yaw);
        theta0 = (float)yaw;

        laser_mutex.lock();
            if(laser_new == true){
                local_laser = current_laser; 
                laser_new = false;
            }   
            else toExit = true;
        laser_mutex.unlock();
        if(toExit == true) return;
        Grid::Point* grid = computeBRT(local_laser);
        float x,y,v,theta;
        odom_mtx.lock();
            if(odom_new == true){
                local_odom = current_odom;
                odom_new = false;
            }
            else toExit = true;
        odom_mtx.unlock();
        if(toExit == true) return;
        x = local_odom.pose.pose.position.x;
        y = local_odom.pose.pose.position.y;
        v = local_odom.twist.twist.linear.x;
        tf2::Quaternion q(local_odom.pose.pose.orientation.w,local_odom.pose.pose.orientation.x,local_odom.pose.pose.orientation.y,local_odom.pose.pose.orientation.z);
        tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
        theta = (float)yaw;

        visualizeReachabilitySetSlice(grid,v,theta-theta0);
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.frame_id = "ego_racecar/base_link";
        msg.header.stamp = rclcpp::Time(0);
        int idx = Grid::getID(x-x0,y-y0,v,theta-theta0);
        msg.drive.acceleration = grid[idx].opt_a;
        msg.drive.speed = v + grid[idx].opt_a*Grid::getHorizon();
        msg.drive.steering_angle = grid[idx].opt_delta;
        drive_pub->publish(msg);
        free(grid);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sdf_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr reach_pub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
    rclcpp::TimerBase::SharedPtr timer;
    sensor_msgs::msg::LaserScan current_laser;
    nav_msgs::msg::Odometry current_odom;
    std::mutex odom_mtx;
    bool odom_new, laser_new;
    std::mutex laser_mutex;
    // bool laser_lock;
    // bool odom_lock;
  };

  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SDFNode>(); 
    // firstInitMem() takes the brunt of the slow cudaMalloc command as it allocates and frees memory to a dummy pointer
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // Callbacks run one at a time
    rclcpp::shutdown();
    return 0;
}
