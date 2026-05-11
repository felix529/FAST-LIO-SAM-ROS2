#ifndef FAST_LIO_SAM_UTILITIES_HPP
#define FAST_LIO_SAM_UTILITIES_HPP

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Eigen>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using PointType = pcl::PointXYZI;

inline gtsam::Pose3 poseEigToGtsamPose(const Eigen::Matrix4d &pose_eig_in)
{
    const Eigen::Matrix3d rot = pose_eig_in.block<3, 3>(0, 0);
    const Eigen::Quaterniond quat_eig(rot);
    tf2::Matrix3x3 mat(tf2::Quaternion(quat_eig.x(), quat_eig.y(), quat_eig.z(), quat_eig.w()));
    double r, p, y;
    mat.getRPY(r, p, y);
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(r, p, y),
                        gtsam::Point3(pose_eig_in(0, 3), pose_eig_in(1, 3), pose_eig_in(2, 3)));
}

inline Eigen::Matrix4d gtsamPoseToPoseEig(const gtsam::Pose3 &gtsam_pose_in)
{
    Eigen::Matrix4d pose_eig_out = Eigen::Matrix4d::Identity();
    tf2::Quaternion quat;
    quat.setRPY(gtsam_pose_in.rotation().roll(),
                gtsam_pose_in.rotation().pitch(),
                gtsam_pose_in.rotation().yaw());
    tf2::Matrix3x3 mat(quat);
    Eigen::Matrix3d tmp_rot_mat;
    tmp_rot_mat << mat[0][0], mat[0][1], mat[0][2],
                   mat[1][0], mat[1][1], mat[1][2],
                   mat[2][0], mat[2][1], mat[2][2];
    pose_eig_out.block<3, 3>(0, 0) = tmp_rot_mat;
    pose_eig_out(0, 3) = gtsam_pose_in.translation().x();
    pose_eig_out(1, 3) = gtsam_pose_in.translation().y();
    pose_eig_out(2, 3) = gtsam_pose_in.translation().z();
    return pose_eig_out;
}

inline geometry_msgs::msg::PoseStamped poseEigToPoseStamped(const Eigen::Matrix4d &pose_eig_in,
                                                            std::string frame_id = "map")
{
    const Eigen::Quaterniond quat_eig(pose_eig_in.block<3, 3>(0, 0));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = pose_eig_in(0, 3);
    pose.pose.position.y = pose_eig_in(1, 3);
    pose.pose.position.z = pose_eig_in(2, 3);
    pose.pose.orientation.w = quat_eig.w();
    pose.pose.orientation.x = quat_eig.x();
    pose.pose.orientation.y = quat_eig.y();
    pose.pose.orientation.z = quat_eig.z();
    return pose;
}

inline geometry_msgs::msg::PoseStamped gtsamPoseToPoseStamped(const gtsam::Pose3 &gtsam_pose_in,
                                                              std::string frame_id = "map")
{
    tf2::Quaternion quat;
    quat.setRPY(gtsam_pose_in.rotation().roll(),
                gtsam_pose_in.rotation().pitch(),
                gtsam_pose_in.rotation().yaw());
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = gtsam_pose_in.translation().x();
    pose.pose.position.y = gtsam_pose_in.translation().y();
    pose.pose.position.z = gtsam_pose_in.translation().z();
    pose.pose.orientation.w = quat.getW();
    pose.pose.orientation.x = quat.getX();
    pose.pose.orientation.y = quat.getY();
    pose.pose.orientation.z = quat.getZ();
    return pose;
}

inline geometry_msgs::msg::TransformStamped poseEigToTransformStamped(const Eigen::Matrix4d &pose,
                                                                      const std::string &parent_frame,
                                                                      const std::string &child_frame,
                                                                      const rclcpp::Time &stamp)
{
    Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = pose(0, 3);
    transform.transform.translation.y = pose(1, 3);
    transform.transform.translation.z = pose(2, 3);
    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();
    return transform;
}

template<typename T>
inline sensor_msgs::msg::PointCloud2 pclToPclRos(pcl::PointCloud<T> cloud,
                                                 std::string frame_id = "map")
{
    sensor_msgs::msg::PointCloud2 cloud_ros;
    pcl::toROSMsg(cloud, cloud_ros);
    cloud_ros.header.frame_id = frame_id;
    return cloud_ros;
}

template<typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose_tf)
{
    if (cloud_in.empty())
    {
        return cloud_in;
    }
    pcl::PointCloud<T> pcl_out = cloud_in;
    pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);
    return pcl_out;
}

inline pcl::PointCloud<pcl::PointXYZI>::Ptr voxelizePcd(const pcl::PointCloud<pcl::PointXYZI> &pcd_in,
                                                        const float voxel_res)
{
    static pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_out(new pcl::PointCloud<pcl::PointXYZI>);
    pcd_in_ptr->reserve(pcd_in.size());
    pcd_out->reserve(pcd_in.size());
    *pcd_in_ptr = pcd_in;
    voxelgrid.setInputCloud(pcd_in_ptr);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

inline pcl::PointCloud<pcl::PointXYZI>::Ptr voxelizePcd(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pcd_in,
                                                        const float voxel_res)
{
    static pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_out(new pcl::PointCloud<pcl::PointXYZI>);
    pcd_out->reserve(pcd_in->size());
    voxelgrid.setInputCloud(pcd_in);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

#endif


