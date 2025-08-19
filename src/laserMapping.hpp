#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

void SigHandle(int sig);
inline void dump_lio_state_to_log(FILE *fp);
void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s);
void pointBodyToWorld(PointType const * const pi, PointType * const po);
template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po);
void RGBpointBodyToWorld(PointType const * const pi, PointType * const po);
void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po);
void points_cache_collect();
void lasermap_fov_segment();
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg);
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg);
void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg);
bool sync_packages(MeasureGroup &meas);
void map_incremental();
void preprocess_lidar();
void publish_lidar();
void publish_imu();
void publish_odometry();
void publish_path();
void publish_map();
void save_to_pcd();
template<typename T>
void set_posestamp(T & out);
void publish_odometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br);
void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

class LaserMappingNode;
//{
//    public:
//        LaserMappingNode(const rclcpp::NodeOptions& options);
//        ~LaserMappingNode();
//    private:
//        void timer_callback();
//        void map_publish_callback();
//        void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
//};