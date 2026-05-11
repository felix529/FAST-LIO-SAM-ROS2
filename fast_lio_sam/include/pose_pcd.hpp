#ifndef FAST_LIO_SAM_POSE_PCD_HPP
#define FAST_LIO_SAM_POSE_PCD_HPP

#include "utilities.hpp"

struct PosePcd
{
    pcl::PointCloud<pcl::PointXYZI> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    double timestamp_;
    int idx_;
    bool processed_ = false;
    PosePcd() {}
    PosePcd(const nav_msgs::msg::Odometry &odom_in,
            const sensor_msgs::msg::PointCloud2 &pcd_in,
            const int &idx_in);
};

inline PosePcd::PosePcd(const nav_msgs::msg::Odometry &odom_in,
                        const sensor_msgs::msg::PointCloud2 &pcd_in,
                        const int &idx_in)
{
    Eigen::Quaterniond q(odom_in.pose.pose.orientation.w,
                         odom_in.pose.pose.orientation.x,
                         odom_in.pose.pose.orientation.y,
                         odom_in.pose.pose.orientation.z);
    pose_eig_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    pose_eig_(0, 3) = odom_in.pose.pose.position.x;
    pose_eig_(1, 3) = odom_in.pose.pose.position.y;
    pose_eig_(2, 3) = odom_in.pose.pose.position.z;
    pose_corrected_eig_ = pose_eig_;
    pcl::PointCloud<pcl::PointXYZI> tmp_pcd;
    pcl::fromROSMsg(pcd_in, tmp_pcd);
    pcd_ = transformPcd(tmp_pcd, pose_eig_.inverse());
    timestamp_ = rclcpp::Time(odom_in.header.stamp).seconds();
    idx_ = idx_in;
}

#endif
