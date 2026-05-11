#ifndef FAST_LIO_SAM_QN_MAIN_H
#define FAST_LIO_SAM_QN_MAIN_H

#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "loop_closure.h"
#include "pose_pcd.hpp"
#include "utilities.hpp"

namespace fs = std::filesystem;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> odom_pcd_sync_pol;

class FastLioSam
{
private:
    std::string map_frame_;
    std::string package_path_;
    std::string seq_name_;
    std::mutex realtime_pose_mutex_, keyframes_mutex_;
    std::mutex graph_mutex_, vis_mutex_;
    Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
    PosePcd current_frame_;
    std::vector<PosePcd> keyframes_;
    int current_keyframe_idx_ = 0;
    bool is_initialized_ = false;
    bool loop_enable_ = false;
    bool global_map_publish_enable_ = true;
    bool loop_added_flag_ = false;
    bool loop_added_flag_vis_ = false;
    std::shared_ptr<gtsam::ISAM2> isam_handler_ = nullptr;
    gtsam::NonlinearFactorGraph gtsam_graph_;
    gtsam::Values init_esti_;
    gtsam::Values corrected_esti_;
    double keyframe_thr_;
    double voxel_res_;
    int sub_key_num_;
    std::vector<std::pair<size_t, size_t>> loop_idx_pairs_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
    nav_msgs::msg::Path odom_path_, corrected_path_;
    bool global_map_vis_switch_ = true;
    size_t last_published_map_keyframe_count_ = 0;
    int global_map_publish_keyframe_interval_ = 5;
    bool save_map_bag_ = false, save_map_pcd_ = false, save_in_kitti_format_ = false;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corrected_odom_pub_, corrected_path_as_cloud_pub_, odom_pub_, path_as_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, corrected_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr corrected_current_pcd_pub_, corrected_pcd_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr loop_detection_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr realtime_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_src_pub_, debug_dst_pub_, debug_fine_aligned_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_save_flag_;
    rclcpp::TimerBase::SharedPtr loop_timer_, vis_timer_;
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> sub_odom_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_pcd_ = nullptr;
    std::shared_ptr<LoopClosure> loop_closure_;

public:
    explicit FastLioSam(const rclcpp::Node::SharedPtr &node);
    ~FastLioSam();

private:
    template<typename T>
    T getParam(const std::string &name, const T &default_value)
    {
        if (!node_->has_parameter(name))
        {
            node_->declare_parameter<T>(name, default_value);
        }
        return node_->get_parameter(name).get_value<T>();
    }

    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    visualization_msgs::msg::Marker getLoopMarkers(const gtsam::Values &corrected_esti_in);
    void odomPcdCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                         const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg);
    void saveFlagCallback(const std_msgs::msg::String::SharedPtr msg);
    void loopTimerFunc();
    void visTimerFunc();
};

#endif

