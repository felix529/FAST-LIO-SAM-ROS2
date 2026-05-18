#include "fast_lio_sam.h"

FastLioSam::FastLioSam(const rclcpp::Node::SharedPtr &node):
    node_(node)
{
    ////// ROS params
    double loop_update_hz, vis_hz;
    LoopClosureConfig lc_config;
    /* basic */
    map_frame_ = getParam<std::string>("basic.map_frame", "map");
    loop_update_hz = getParam<double>("basic.loop_update_hz", 1.0);
    vis_hz = getParam<double>("basic.vis_hz", 0.5);
    global_map_publish_enable_ = getParam<bool>("basic.publish_global_map", true);
    global_map_publish_keyframe_interval_ = getParam<int>("basic.global_map_publish_keyframe_interval", 5);
    /* keyframe */
    keyframe_thr_ = getParam<double>("keyframe.keyframe_threshold", 1.0);
    lc_config.num_submap_keyframes_ = getParam<int>("keyframe.nusubmap_keyframes", 5);
    /* loop */
    loop_enable_ = getParam<bool>("loop.enable", false);
    lc_config.loop_detection_radius_ = getParam<double>("loop.loop_detection_radius", 15.0);
    lc_config.loop_detection_timediff_threshold_ = getParam<double>("loop.loop_detection_timediff_threshold", 10.0);
    lc_config.icp_max_corr_dist_ = lc_config.loop_detection_radius_ * 1.5;
    /* icp */
    lc_config.voxel_res_ = getParam<double>("icp.icp_voxel_resolution", 0.3);
    lc_config.icp_score_threshold_ = getParam<double>("icp.icp_score_threshold", 0.3);
    /* results */
    voxel_res_ = getParam<double>("result.save_voxel_resolution", 0.3);
    save_map_pcd_ = getParam<bool>("result.save_map_pcd", false);
    save_map_bag_ = getParam<bool>("result.save_map_bag", false);
    save_in_kitti_format_ = getParam<bool>("result.save_in_kitti_format", false);
    seq_name_ = getParam<std::string>("result.seq_name", "");
    /* Loop closure */
    loop_closure_.reset(new LoopClosure(lc_config));
    /* Initialization of GTSAM */
    gtsam::ISAM2Params isam_params_;
    isam_params_.relinearizeThreshold = 0.01;
    isam_params_.relinearizeSkip = 1;
    isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params_);
    /* ROS things */
    odom_path_.header.frame_id = map_frame_;
    corrected_path_.header.frame_id = map_frame_;
    package_path_ = ament_index_cpp::get_package_share_directory("fast_lio_sam");
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    auto latched_qos = rclcpp::QoS(10).transient_local();
    /* publishers */
    odom_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/ori_odom", latched_qos);
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/ori_path", latched_qos);
    corrected_odom_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_odom", latched_qos);
    corrected_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/corrected_path", latched_qos);
    corrected_pcd_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_map", latched_qos);
    corrected_current_pcd_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_current_pcd", latched_qos);
    loop_detection_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/loop_detection", latched_qos);
    realtime_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_stamped", rclcpp::QoS(10));
    debug_src_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/src", latched_qos);
    debug_dst_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/dst", latched_qos);
    debug_fine_aligned_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned", latched_qos);
    /* subscribers */
    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(node_, "/Odometry", rmw_qos_profile_sensor_data);
    sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(node_, "/cloud_registered", rmw_qos_profile_sensor_data);
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(std::bind(&FastLioSam::odomPcdCallback, this, std::placeholders::_1, std::placeholders::_2));
    sub_save_flag_ = node_->create_subscription<std_msgs::msg::String>("/save_dir", 10, std::bind(&FastLioSam::saveFlagCallback, this, std::placeholders::_1));
    /* Timers */
    loop_timer_ = node_->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / loop_update_hz)), std::bind(&FastLioSam::loopTimerFunc, this));
    vis_timer_ = node_->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / vis_hz)), std::bind(&FastLioSam::visTimerFunc, this));
    RCLCPP_INFO(node_->get_logger(), "Main class, starting node...");
}

void FastLioSam::odomPcdCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                                 const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg)
{
    Eigen::Matrix4d last_odom_tf;
    last_odom_tf = current_frame_.pose_eig_;                              // to calculate delta
    current_frame_ = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_); // to be checked if keyframe or not
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    {
        //// 1. realtime pose = last corrected odom * delta (last -> current)
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
        current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
        realtime_pose_pub_->publish(poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_));
        broadcaster_->sendTransform(poseEigToTransformStamped(current_frame_.pose_corrected_eig_, map_frame_, "robot", node_->now()));
    }
    corrected_current_pcd_pub_->publish(pclToPclRos(transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_), map_frame_));

    if (!is_initialized_) //// init only once
    {
        // others
        keyframes_.push_back(current_frame_);
        updateOdomsAndPaths(current_frame_);
        // graph
        auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished(); // rad*rad,
                                                                                                    // meter*meter
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseEigToGtsamPose(current_frame_.pose_eig_), prior_noise));
        init_esti_.insert(current_keyframe_idx_, poseEigToGtsamPose(current_frame_.pose_eig_));
        current_keyframe_idx_++;
        is_initialized_ = true;
    }
    else
    {
        //// 2. check if keyframe
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        if (checkIfKeyframe(current_frame_, keyframes_.back()))
        {
            // 2-2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                keyframes_.push_back(current_frame_);
            }
            // 2-3. if so, add to graph
            auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
            gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
            gtsam::Pose3 pose_from = poseEigToGtsamPose(keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
            gtsam::Pose3 pose_to = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(current_keyframe_idx_ - 1,
                                                                    current_keyframe_idx_,
                                                                    pose_from.between(pose_to),
                                                                    odom_noise));
                init_esti_.insert(current_keyframe_idx_, pose_to);
            }
            current_keyframe_idx_++;

            //// 3. vis
            high_resolution_clock::time_point t3 = high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame_);
            }

            //// 4. optimize with graph
            high_resolution_clock::time_point t4 = high_resolution_clock::now();
            // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, init_esti_).optimize(); // cf. isam.update vs values.LM.optimize
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                isam_handler_->update(gtsam_graph_, init_esti_);
                isam_handler_->update();
                if (loop_added_flag_) // https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
                {
                    isam_handler_->update();
                    isam_handler_->update();
                    isam_handler_->update();
                }
                gtsam_graph_.resize(0);
                init_esti_.clear();
            }

            //// 5. handle corrected results
            // get corrected poses and reset odom delta (for realtime pose pub)
            high_resolution_clock::time_point t5 = high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
                corrected_esti_ = isam_handler_->calculateEstimate();
                last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
                odom_delta_ = Eigen::Matrix4d::Identity();
            }
            // correct poses in keyframes
            if (loop_added_flag_)
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                for (size_t i = 0; i < corrected_esti_.size(); ++i)
                {
                    keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
                }
                loop_added_flag_ = false;
            }
            high_resolution_clock::time_point t6 = high_resolution_clock::now();

            RCLCPP_INFO(node_->get_logger(), "real: %.1f, key_add: %.1f, vis: %.1f, opt: %.1f, res: %.1f, tot: %.1fms",
                     duration_cast<microseconds>(t2 - t1).count() / 1e3,
                     duration_cast<microseconds>(t3 - t2).count() / 1e3,
                     duration_cast<microseconds>(t4 - t3).count() / 1e3,
                     duration_cast<microseconds>(t5 - t4).count() / 1e3,
                     duration_cast<microseconds>(t6 - t5).count() / 1e3,
                     duration_cast<microseconds>(t6 - t1).count() / 1e3);
        }
    }
    return;
}

void FastLioSam::loopTimerFunc()
{
    if (!loop_enable_)
    {
        return;
    }
    if (!is_initialized_ || keyframes_.empty())
    {
        return;
    }
    auto &latest_keyframe = keyframes_.back();
    if (latest_keyframe.processed_)
    {
        return;
    }
    latest_keyframe.processed_ = true;

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    const int closest_keyframe_idx = loop_closure_->fetchClosestKeyframeIdx(latest_keyframe, keyframes_);
    if (closest_keyframe_idx < 0)
    {
        return;
    }

    const RegistrationOutput &reg_output = loop_closure_->performLoopClosure(latest_keyframe, keyframes_, closest_keyframe_idx);
    if (reg_output.is_valid_)
    {
        RCLCPP_INFO(node_->get_logger(), "\033[1;32mLoop closure accepted. Score: %.3f\033[0m", reg_output.score_);
        const auto &score = reg_output.score_;
        gtsam::Pose3 pose_from = poseEigToGtsamPose(reg_output.pose_between_eig_ * latest_keyframe.pose_corrected_eig_); // IMPORTANT: take care of the order
        gtsam::Pose3 pose_to = poseEigToGtsamPose(keyframes_[closest_keyframe_idx].pose_corrected_eig_);
        auto variance_vector = (gtsam::Vector(6) << score, score, score, score, score, score).finished();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(latest_keyframe.idx_,
                                                                closest_keyframe_idx,
                                                                pose_from.between(pose_to),
                                                                loop_noise));
        }
        loop_idx_pairs_.push_back({latest_keyframe.idx_, closest_keyframe_idx}); // for vis
        loop_added_flag_vis_ = true;
        loop_added_flag_ = true;
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Loop closure rejected. Score: %.3f", reg_output.score_);
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    debug_src_pub_->publish(pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
    debug_dst_pub_->publish(pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
    debug_fine_aligned_pub_->publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));

    RCLCPP_INFO(node_->get_logger(), "loop: %.1f", duration_cast<microseconds>(t2 - t1).count() / 1e3);
    return;
}

void FastLioSam::visTimerFunc()
{
    if (!is_initialized_)
    {
        return;
    }

    high_resolution_clock::time_point tv1 = high_resolution_clock::now();
    //// 1. if loop closed, correct vis data
    if (loop_added_flag_vis_)
    // copy and ready
    {
        gtsam::Values corrected_esti_copied;
        pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
        nav_msgs::msg::Path corrected_path;
        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            corrected_esti_copied = corrected_esti_;
        }
        // correct pose and path
        for (size_t i = 0; i < corrected_esti_copied.size(); ++i)
        {
            gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
            corrected_odoms.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
            corrected_path.poses.push_back(gtsamPoseToPoseStamped(pose_, map_frame_));
        }
        // update vis of loop constraints
        if (!loop_idx_pairs_.empty())
        {
            loop_detection_pub_->publish(getLoopMarkers(corrected_esti_copied));
        }
        // update with corrected data
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_ = corrected_odoms;
            corrected_path_.poses = corrected_path.poses;
        }
        loop_added_flag_vis_ = false;
    }
    //// 2. publish odoms, paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        odom_pub_->publish(pclToPclRos(odoms_, map_frame_));
        path_pub_->publish(odom_path_);
        corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_->publish(corrected_path_);
    }

    //// 3. global map
    const size_t keyframe_count = keyframes_.size();
    const bool should_publish_new_map =
        global_map_vis_switch_ ||
        (global_map_publish_keyframe_interval_ > 0 &&
         keyframe_count >= last_published_map_keyframe_count_ + static_cast<size_t>(global_map_publish_keyframe_interval_));
    if (global_map_publish_enable_ &&
        corrected_pcd_map_pub_->get_subscription_count() > 0 &&
        keyframe_count > 0 &&
        should_publish_new_map)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframe_count); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        corrected_pcd_map_pub_->publish(pclToPclRos(*voxelized_map, map_frame_));
        global_map_vis_switch_ = false;
        last_published_map_keyframe_count_ = keyframe_count;
    }
    if (!global_map_vis_switch_ && corrected_pcd_map_pub_->get_subscription_count() == 0)
    {
        global_map_vis_switch_ = true;
        last_published_map_keyframe_count_ = 0;
    }
    high_resolution_clock::time_point tv2 = high_resolution_clock::now();
    RCLCPP_INFO(node_->get_logger(), "vis: %.1fms", duration_cast<microseconds>(tv2 - tv1).count() / 1e3);
    return;
}

void FastLioSam::saveFlagCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string save_dir = msg->data != "" ? msg->data : package_path_;

    // save scans as individual pcd files and poses in KITTI format
    // Delete the scans folder if it exists and create a new one
    std::string seq_directory = save_dir + "/" + seq_name_;
    std::string scans_directory = seq_directory + "/scans";
    if (save_in_kitti_format_)
    {
        RCLCPP_INFO(node_->get_logger(), "\033[32;1mScans are saved in %s, following the KITTI and TUM format\033[0m", scans_directory.c_str());
        if (fs::exists(seq_directory))
        {
            fs::remove_all(seq_directory);
        }
        fs::create_directories(scans_directory);

        std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
        std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
        tum_pose_file << "#timestamp x y z qx qy qz qw\n";
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                // Save the point cloud
                std::stringstream ss_;
                ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
                RCLCPP_INFO(node_->get_logger(), "Saving %s...", ss_.str().c_str());
                pcl::io::savePCDFileASCII<PointType>(ss_.str(), keyframes_[i].pcd_);

                // Save the pose in KITTI format
                const auto &pose_ = keyframes_[i].pose_corrected_eig_;
                kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " " << pose_(0, 2) << " "
                                << pose_(0, 3) << " " << pose_(1, 0) << " " << pose_(1, 1) << " "
                                << pose_(1, 2) << " " << pose_(1, 3) << " " << pose_(2, 0) << " "
                                << pose_(2, 1) << " " << pose_(2, 2) << " " << pose_(2, 3) << "\n";

                const auto &lidar_optim_pose_ = poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_);
                tum_pose_file << std::fixed << std::setprecision(8) << keyframes_[i].timestamp_
                              << " " << lidar_optim_pose_.pose.position.x << " "
                              << lidar_optim_pose_.pose.position.y << " "
                              << lidar_optim_pose_.pose.position.z << " "
                              << lidar_optim_pose_.pose.orientation.x << " "
                              << lidar_optim_pose_.pose.orientation.y << " "
                              << lidar_optim_pose_.pose.orientation.z << " "
                              << lidar_optim_pose_.pose.orientation.w << "\n";
            }
        }
        kitti_pose_file.close();
        tum_pose_file.close();
        RCLCPP_INFO(node_->get_logger(), "\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m");
    }

    if (save_map_bag_)
    {
        RCLCPP_WARN(node_->get_logger(), "save_map_bag is not implemented in this ROS 2 port; use ros2 bag record for bag output.");
    }

    if (save_map_pcd_)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(seq_directory + "/" + seq_name_ + "_map.pcd", *voxelized_map);
        RCLCPP_INFO(node_->get_logger(), "\033[32;1mAccumulated map cloud saved in .pcd format\033[0m");
    }
}

FastLioSam::~FastLioSam()
{
    // save map
    if (save_map_bag_)
    {
        RCLCPP_WARN(node_->get_logger(), "save_map_bag is not implemented in this ROS 2 port; use ros2 bag record for bag output.");
    }
}

void FastLioSam::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                               pose_pcd_in.pose_eig_(1, 3),
                               pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
}

visualization_msgs::msg::Marker FastLioSam::getLoopMarkers(const gtsam::Values &corrected_esti_in)
{
    visualization_msgs::msg::Marker edges;
    edges.type = 5u;
    edges.scale.x = 0.12f;
    edges.header.frame_id = map_frame_;
    edges.pose.orientation.w = 1.0f;
    edges.color.r = 1.0f;
    edges.color.g = 1.0f;
    edges.color.b = 1.0f;
    edges.color.a = 1.0f;
    for (size_t i = 0; i < loop_idx_pairs_.size(); ++i)
    {
        if (loop_idx_pairs_[i].first >= corrected_esti_in.size() ||
            loop_idx_pairs_[i].second >= corrected_esti_in.size())
        {
            continue;
        }
        gtsam::Pose3 pose = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].first);
        gtsam::Pose3 pose2 = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].second);
        geometry_msgs::msg::Point p, p2;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        p2.x = pose2.translation().x();
        p2.y = pose2.translation().y();
        p2.z = pose2.translation().z();
        edges.points.push_back(p);
        edges.points.push_back(p2);
    }
    return edges;
}

bool FastLioSam::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    return keyframe_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}



