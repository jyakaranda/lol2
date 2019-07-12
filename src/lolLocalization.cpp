/**
 * @file lolLocalization.cpp
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-12
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "lol2/lolLocalization.h"

namespace localization
{
LolLocalization::LolLocalization() : Node("lol")
{
  this->get_parameter_or("surround_search_radius", surround_search_radius_, 50.0);
  this->get_parameter_or("surround_search_radius", surround_search_radius_, 50.0);
  this->get_parameter_or("surround_search_num", surround_search_num_, 50);
  this->get_parameter_or("corner_leaf", corner_leaf_, 0.2);
  this->get_parameter_or("surf_leaf", surf_leaf_, 0.5);
  this->get_parameter_or("outlier_leaf", outlier_leaf_, 0.5);
  this->get_parameter_or("surround_keyposes_leaf", surround_keyposes_leaf_, 1.0);
  this->get_parameter_or("fn_poses", fn_poses_, std::string("~"));
  this->get_parameter_or("fn_corner", fn_corner_, std::string("~"));
  this->get_parameter_or("fn_surf", fn_surf_, std::string("~"));
  this->get_parameter_or("fn_outlier", fn_outlier_, std::string("~"));
  this->get_parameter_or("target_update_dist", target_update_dist_, 5.);
  this->get_parameter_or("max_opt_iters", max_opt_iters_, 4);
  this->get_parameter_or("batch_size", batch_size_, 5);
  this->get_parameter_or("huber_s", huber_s_, 0.2);
  this->get_parameter_or("max_iters", max_iters_, 10);
  this->get_parameter_or("func_tolerance", func_tolerance_, 1e-6);
  this->get_parameter_or("gradient_tolerance", gradient_tolerance_, 1e-10);
  this->get_parameter_or("param_tolerance", param_tolerance_, 1e-8);

  double roll, pitch, yaw;
  if (!this->get_parameter("tf_b2l_x", tf_b2l_(0, 3)) || !this->get_parameter("tf_b2l_y", tf_b2l_(1, 3)) || !this->get_parameter("tf_b2l_z", tf_b2l_(2, 3)) || !this->get_parameter("tf_b2l_roll", roll) || !this->get_parameter("tf_b2l_pitch", pitch) || !this->get_parameter("tf_b2l_yaw", yaw))
  {
    RCLCPP_ERROR(this->get_logger(), "transform between /base_link to /laser not set.");
    exit(-1);
  }
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  tf_b2l_.block(0, 0, 3, 3) = (rz * ry * rx).matrix();

  if (!init())
  {
    RCLCPP_ERROR(this->get_logger(), "failed init.");
    exit(-1);
  }

  pub_lol_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose");
  pub_corner_target_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/corner_target");
  pub_surf_target_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/surf_target");
  pub_corner_source_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/corner_source");
  pub_surf_source_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/surf_source");
  pub_test_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/test_pc");
  pub_confidence_ = this->create_publisher<std_msgs::msg::Float32>("/lol_confidence");

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/lidar", std::bind(&LolLocalization::odomCB, this, std::placeholders::_1));
  sub_corner_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/corner", std::bind(&LolLocalization::cornerCB, this, std::placeholders::_1));
  sub_surf_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/surf", std::bind(&LolLocalization::surfCB, this, std::placeholders::_1));
  sub_outlier_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/outlier", std::bind(&LolLocalization::outlierCB, this, std::placeholders::_1));
  sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", std::bind(&LolLocalization::initialPoseCB, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "lol init ok.");
}

bool LolLocalization::init()
{
  TicToc t_init;
  keyposes_3d_.reset(new pcl::PointCloud<PointType>());

  laser_corner_.reset(new pcl::PointCloud<PointType>());
  laser_surf_.reset(new pcl::PointCloud<PointType>());
  laser_outlier_.reset(new pcl::PointCloud<PointType>());
  laser_corner_ds_.reset(new pcl::PointCloud<PointType>());
  laser_surf_ds_.reset(new pcl::PointCloud<PointType>());
  laser_outlier_ds_.reset(new pcl::PointCloud<PointType>());
  pc_corner_target_.reset(new pcl::PointCloud<PointType>());
  pc_surf_target_.reset(new pcl::PointCloud<PointType>());
  pc_corner_target_ds_.reset(new pcl::PointCloud<PointType>());
  pc_surf_target_ds_.reset(new pcl::PointCloud<PointType>());

  kdtree_keyposes_3d_.reset(new pcl::KdTreeFLANN<PointType>());
  kdtree_corner_target_.reset(new pcl::KdTreeFLANN<PointType>());
  kdtree_surf_target_.reset(new pcl::KdTreeFLANN<PointType>());

  ds_corner_.setLeafSize(corner_leaf_, corner_leaf_, corner_leaf_);
  ds_surf_.setLeafSize(surf_leaf_, surf_leaf_, surf_leaf_);
  ds_outlier_.setLeafSize(outlier_leaf_, outlier_leaf_, outlier_leaf_);
  ds_surround_keyposes_.setLeafSize(surround_keyposes_leaf_, surround_keyposes_leaf_, surround_keyposes_leaf_);

  target_center_.x = target_center_.y = target_center_.z = -100.;
  cur_laser_pose_ = geometry_msgs::msg::PoseStamped();
  pre_laser_pose_ = geometry_msgs::msg::PoseStamped();

  // 加载地图和位姿 pcd 文件
  pcl::PointCloud<PointType>::Ptr corner_pc(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surf_pc(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr outlier_pc(new pcl::PointCloud<PointType>());

  TicToc t_data;
  if (pcl::io::loadPCDFile(fn_poses_, *keyposes_3d_) == -1 || pcl::io::loadPCDFile(fn_corner_, *corner_pc) == -1 ||
      pcl::io::loadPCDFile(fn_surf_, *surf_pc) == -1 || pcl::io::loadPCDFile(fn_outlier_, *outlier_pc) == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "couldn't load pcd file");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "time: %f s ----> keyposes: %d, corner pc: %d, surf pc: %d, outlier pc: %d", t_data.toc(), keyposes_3d_->points.size(), corner_pc->points.size(), surf_pc->points.size(), outlier_pc->points.size());

  kdtree_keyposes_3d_->setInputCloud(keyposes_3d_);

  corner_keyframes_.resize(keyposes_3d_->points.size());
  surf_keyframes_.resize(keyposes_3d_->points.size());
  outlier_keyframes_.resize(keyposes_3d_->points.size());
  for (int i = 0; i < keyposes_3d_->points.size(); ++i)
  {
    corner_keyframes_[i] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    surf_keyframes_[i] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    outlier_keyframes_[i] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
  }

  for (int i = 0; i < corner_pc->points.size(); ++i)
  {
    const auto &p = corner_pc->points[i];
    corner_keyframes_[int(p.intensity)]->points.push_back(p);
  }
  for (int i = 0; i < surf_pc->points.size(); ++i)
  {
    const auto &p = surf_pc->points[i];
    surf_keyframes_[int(p.intensity)]->points.push_back(p);
  }
  for (int i = 0; i < outlier_pc->points.size(); ++i)
  {
    const auto &p = outlier_pc->points[i];
    outlier_keyframes_[int(p.intensity)]->points.push_back(p);
  }

  time_laser_corner_ = time_laser_surf_ = time_laser_outlier_ = time_laser_odom_ = 0.;
  new_laser_corner_ = new_laser_surf_ = new_laser_outlier_ = new_laser_odom_ = false;

  batch_cnt_ = 0;
  local_corner_.reset(new PointCloudT);
  local_surf_.reset(new PointCloudT);
  local_outlier_.reset(new PointCloudT);

  tf_o2l_0_ = Eigen::Matrix4d::Identity();
  tf_m2l_0_ = Eigen::Matrix4d::Identity();

  tf_m2o_ = Eigen::Matrix4d::Identity();
  tf_o2b_ = Eigen::Matrix4d::Identity();
  tf_b2l_ = Eigen::Matrix4d::Identity();
  tf_m2o_update_ = Eigen::Matrix4d::Identity();

  for (int i = 0; i < 6; ++i)
  {
    tobe_optimized_[i] = 0;
  }

  lol_confidence_ = 80.;

  RCLCPP_INFO(this->get_logger(), "init ok. time: %.3fms", t_init.toc());

  return true;
}

void LolLocalization::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // update
  tf_o2b_.block<3, 3>(0, 0) = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
  tf_o2b_(0, 3) = msg->pose.pose.position.x;
  tf_o2b_(1, 3) = msg->pose.pose.position.y;
  tf_o2b_(2, 3) = msg->pose.pose.position.z;

  time_laser_odom_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  new_laser_odom_ = true;

  // publish
  Eigen::Quaterniond tmp_q(tf_m2o_.block<3, 3>(0, 0));
  geometry_msgs::msg::TransformStamped msg_m2o;
  msg_m2o.header.stamp = msg->header.stamp;
  msg_m2o.header.frame_id = "map";
  msg_m2o.child_frame_id = "/odom";
  msg_m2o.transform.rotation.w = tmp_q.w();
  msg_m2o.transform.rotation.x = tmp_q.x();
  msg_m2o.transform.rotation.y = tmp_q.y();
  msg_m2o.transform.rotation.z = tmp_q.z();
  msg_m2o.transform.translation.x = tf_m2o_(0, 3);
  msg_m2o.transform.translation.y = tf_m2o_(1, 3);
  msg_m2o.transform.translation.z = tf_m2o_(2, 3);
  tf_broad_->sendTransform(msg_m2o);

  Eigen::Matrix4d m2b = tf_m2o_ * tf_o2b_;
  tmp_q = m2b.block<3, 3>(0, 0);
  geometry_msgs::msg::PoseStamped msg_pose;
  msg_pose.header.frame_id = "/map";
  msg_pose.header.stamp = msg->header.stamp;
  msg_pose.pose.orientation.w = tmp_q.w();
  msg_pose.pose.orientation.x = tmp_q.x();
  msg_pose.pose.orientation.y = tmp_q.y();
  msg_pose.pose.orientation.z = tmp_q.z();
  msg_pose.pose.position.x = m2b(0, 3);
  msg_pose.pose.position.y = m2b(1, 3);
  msg_pose.pose.position.z = m2b(2, 3);
  pub_lol_pose_->publish(msg_pose);

  std_msgs::msg::Float32 msg_conf;
  msg_conf.data = lol_confidence_;
  pub_confidence_->publish(msg_conf);
}

void LolLocalization::cornerCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  laser_corner_->clear();
  pcl::fromROSMsg(*msg, *laser_corner_);
  time_laser_corner_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  new_laser_corner_ = true;
}
void LolLocalization::surfCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  laser_surf_->clear();
  pcl::fromROSMsg(*msg, *laser_surf_);
  time_laser_surf_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  new_laser_surf_ = true;
}
void LolLocalization::outlierCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  laser_outlier_->clear();
  pcl::fromROSMsg(*msg, *laser_outlier_);
  time_laser_outlier_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  new_laser_outlier_ = true;
}

void LolLocalization::initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  mtx_.lock();
  PointType p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  pre_laser_pose_.pose = cur_laser_pose_.pose = msg->pose.pose;
  pre_laser_pose_.header = cur_laser_pose_.header = msg->header;
  extractSurroundKeyFrames(p);

  Eigen::Matrix4d m2b(Eigen::Matrix4d::Identity());
  m2b.block<3, 3>(0, 0) = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
  m2b(0, 3) = msg->pose.pose.position.x;
  m2b(1, 3) = msg->pose.pose.position.y;
  m2b(2, 3) = msg->pose.pose.position.z;
  tf_m2o_ = m2b * tf_o2b_.inverse();
  tf_m2o_update_ = tf_m2o_;

  tf_m2l_0_ = m2b * tf_b2l_;

  batch_cnt_ = 0;
  mtx_.unlock();
  RCLCPP_WARN(this->get_logger(), "init pose set.");
}

bool LolLocalization::extractSurroundKeyFrames(const PointType &p)
{
  RCLCPP_INFO(this->get_logger(), "extract surround keyframes");
  TicToc t_extract;

  pcl::PointCloud<PointType>::Ptr tmp_keyposes(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr tmp_keyposes_ds(new pcl::PointCloud<PointType>());
  kdtree_keyposes_3d_->radiusSearch(p, surround_search_radius_, point_search_idx_, point_search_dist_, 0);
  for (int i = 0; i < point_search_idx_.size(); ++i)
  {
    tmp_keyposes->points.push_back(keyposes_3d_->points[point_search_idx_[i]]);
  }
  ds_surround_keyposes_.setInputCloud(tmp_keyposes);
  ds_surround_keyposes_.filter(*tmp_keyposes_ds);

  bool existing_flag = false;
  // 1. 剔除多余位姿和点云
  for (int i = 0; i < surround_keyposes_id_.size(); ++i)
  {
    existing_flag = false;
    for (int j = 0; j < tmp_keyposes_ds->points.size(); ++j)
    {
      if (int(tmp_keyposes_ds->points[j].intensity) == surround_keyposes_id_[i])
      {
        existing_flag = true;
        break;
      }
    }

    if (!existing_flag)
    {
      surround_keyposes_id_.erase(surround_keyposes_id_.begin() + i);
      surround_corner_keyframes_.erase(surround_corner_keyframes_.begin() + i);
      surround_surf_keyframes_.erase(surround_surf_keyframes_.begin() + i);
      surround_outlier_keyframes_.erase(surround_outlier_keyframes_.begin() + i);
      --i;
    }
  }

  // 2. 添加缺少的位姿和点云
  for (int i = 0; i < tmp_keyposes_ds->points.size(); ++i)
  {
    existing_flag = false;
    int pose_id = int(tmp_keyposes_ds->points[i].intensity);
    for (int j = 0; j < surround_keyposes_id_.size(); ++j)
    {
      if (pose_id == surround_keyposes_id_[j])
      {
        existing_flag = true;
        break;
      }
    }

    if (!existing_flag)
    {
      surround_keyposes_id_.push_back(pose_id);
      surround_corner_keyframes_.push_back(corner_keyframes_[pose_id]);
      surround_surf_keyframes_.push_back(surf_keyframes_[pose_id]);
      surround_outlier_keyframes_.push_back(outlier_keyframes_[pose_id]);
    }
  }

  pc_corner_target_->clear();
  pc_surf_target_->clear();
  pc_corner_target_ds_->clear();
  pc_surf_target_ds_->clear();

  for (int i = 0; i < surround_keyposes_id_.size(); ++i)
  {
    *pc_corner_target_ += *(surround_corner_keyframes_[i]);
    *pc_surf_target_ += *(surround_surf_keyframes_[i]);
    *pc_surf_target_ += *(surround_outlier_keyframes_[i]);
  }

  ds_corner_.setInputCloud(pc_corner_target_);
  ds_corner_.filter(*pc_corner_target_ds_);
  ds_surf_.setInputCloud(pc_surf_target_);
  ds_surf_.filter(*pc_surf_target_ds_);

  if (pub_surf_target_->get_publisher_handle() > 0 || pub_corner_target_->get_publisher_handle() > 0)
  {
    sensor_msgs::msg::PointCloud2 msg_corner_target, msg_surf_target;
    pcl::toROSMsg(*pc_corner_target_ds_, msg_corner_target);
    pcl::toROSMsg(*pc_surf_target_ds_, msg_surf_target);
    msg_corner_target.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    msg_corner_target.header.frame_id = "map";
    msg_surf_target.header = msg_corner_target.header;
    pub_corner_target_->publish(msg_corner_target);
    pub_surf_target_->publish(msg_surf_target);
  }

  target_center_.x = p.x;
  target_center_.y = p.y;

  if (0 == pc_corner_target_ds_->size() || 0 == pc_surf_target_ds_->size())
  {
    RCLCPP_WARN(this->get_logger(), "empty corner or surf target, reset robot pose");
    return false;
  }

  TicToc t_kd;
  kdtree_corner_target_->setInputCloud(pc_corner_target_ds_);
  kdtree_surf_target_->setInputCloud(pc_surf_target_ds_);
  RCLCPP_INFO(this->get_logger(), "kd prepare time: %.3fms, batch_cnt_: %d", t_kd.toc(), batch_cnt_);

  RCLCPP_INFO(this->get_logger(), "time: %.3fms, pc_corner_target_ds_: %d, pc_surf_target_ds_: %d", t_extract.toc(), pc_corner_target_ds_->points.size(), pc_surf_target_ds_->points.size());
  return true;
}

void LolLocalization::run(std::shared_ptr<LolLocalization> lol)
{
  self_ = lol;
  opt_thread_ = std::thread(&localization::LolLocalization::optimizeThread, this);

  rclcpp::Rate rate(100);
  PointCloudT::Ptr test_pc(new PointCloudT);

  while (rclcpp::ok())
  {
    std::cout << "1234" << std::endl;
    rate.sleep();
    rclcpp::spin_some(self_);
    if (new_laser_corner_ && new_laser_surf_ && new_laser_outlier_ && new_laser_odom_ && std::abs(time_laser_odom_ - time_laser_corner_) < 0.005 && std::abs(time_laser_odom_ - time_laser_surf_) < 0.005 && std::abs(time_laser_odom_ - time_laser_outlier_) < 0.005)
    {
      new_laser_corner_ = new_laser_surf_ = new_laser_outlier_ = new_laser_odom_ = false;
      Eigen::Matrix4d tf_m2l = tf_m2o_ * tf_o2b_ * tf_b2l_;

      // stacks a number of scans for batch processing
      if (0 == batch_cnt_)
      {
        if (tf_m2o_ != tf_m2o_update_)
        {
          tf_m2l = tf_m2o_update_ * tf_m2o_.inverse() * tf_m2l;
          tf_m2o_ = tf_m2o_update_;
        }
        // save t0 laser pose as tobe_optimized target
        tf_m2l_0_ = tf_m2l;
        tf_o2l_0_ = tf_o2b_ * tf_b2l_;

        if (pub_test_->get_publisher_handle() > 0)
        {
          sensor_msgs::msg::PointCloud2 test1;
          pcl::toROSMsg(*test_pc, test1);
          test1.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
          test1.header.frame_id = "map";
          pub_test_->publish(test1);
          test_pc->clear();
        }
      }
      // transform feature points to t0 frame
      PointCloudT::Ptr tf_corner(new PointCloudT);
      PointCloudT::Ptr tf_surf(new PointCloudT);
      PointCloudT::Ptr tf_outlier(new PointCloudT);
      pcl::transformPointCloud<PointT, double>(*laser_corner_, *tf_corner, tf_m2l);
      pcl::transformPointCloud<PointT, double>(*laser_surf_, *tf_surf, tf_m2l);
      pcl::transformPointCloud<PointT, double>(*laser_outlier_, *tf_outlier, tf_m2l);
      if (pub_test_->get_publisher_handle() > 0)
      {
        PointCloudT::Ptr tmp(new PointCloudT);
        pcl::transformPointCloud<PointT, double>(*laser_surf_, *tmp, tf_m2l);
        *test_pc += *tmp;
      }

      mtx_.lock();
      ++batch_cnt_;
      *local_corner_ += *tf_corner;
      *local_surf_ += *tf_surf;
      // *local_outlier_ += *tf_outlier;
      *local_surf_ += *tf_outlier;
      mtx_.unlock();
    }
  }
}

void LolLocalization::optimizeThread()
{
  rclcpp::Rate rate(100);

  RCLCPP_INFO(this->get_logger(), "start optimize.");

  while (rclcpp::ok())
  {
    rate.sleep();
    // rclcpp::spin_some(self_);

    TicToc t_all;
    mtx_.lock();
    if (batch_cnt_ < batch_size_)
    {
      mtx_.unlock();
      continue;
    }

    batch_cnt_ = 0; // reset
    // downsize corner, surf, outlier
    TicToc t_ds;
    PointCloudT::Ptr corner_ds(new PointCloudT);
    PointCloudT::Ptr surf_ds(new PointCloudT);
    ds_corner_.setInputCloud(local_corner_);
    ds_corner_.filter(*corner_ds);
    ds_surf_.setInputCloud(local_surf_);
    ds_surf_.filter(*surf_ds);
    RCLCPP_INFO(this->get_logger(), "time: %.3fms, corner_ds: %d, surf_ds: %d", t_ds.toc(), corner_ds->size(), surf_ds->size());
    local_corner_->clear();
    local_surf_->clear();
    // local_outlier_->clear();

    Eigen::Matrix4d m2l_0_backup(tf_m2l_0_);
    Eigen::Matrix4d o2l_o_backup(tf_o2l_0_);

    mtx_.unlock();

    // transform back to t0 local frame, maybe can be optimized
    PointCloudT::Ptr tmp_corner(new PointCloudT);
    PointCloudT::Ptr tmp_surf(new PointCloudT);
    pcl::transformPointCloud<PointT, double>(*corner_ds, *tmp_corner, m2l_0_backup.inverse());
    pcl::transformPointCloud<PointT, double>(*surf_ds, *tmp_surf, m2l_0_backup.inverse());
    corner_ds = tmp_corner;
    surf_ds = tmp_surf;

    if (pub_surf_source_->get_publisher_handle() > 0 || pub_corner_source_->get_publisher_handle() > 0)
    {
      sensor_msgs::msg::PointCloud2 corner_source, surf_source;

      pcl::toROSMsg(*tmp_corner, corner_source);
      pcl::toROSMsg(*tmp_surf, surf_source);
      corner_source.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
      corner_source.header.frame_id = "/laser";
      surf_source.header = corner_source.header;
      pub_corner_source_->publish(corner_source);
      pub_surf_source_->publish(surf_source);
    }

    Eigen::Quaterniond q_m2l(m2l_0_backup.block<3, 3>(0, 0));
    tf2::Matrix3x3(tf2::Quaternion(q_m2l.x(), q_m2l.y(), q_m2l.z(), q_m2l.w())).getEulerYPR(tobe_optimized_[5], tobe_optimized_[4], tobe_optimized_[3]);
    tobe_optimized_[0] = m2l_0_backup(0, 3);
    tobe_optimized_[1] = m2l_0_backup(1, 3);
    tobe_optimized_[2] = m2l_0_backup(2, 3);

    PointT cur_pose;
    cur_pose.x = tobe_optimized_[0];
    cur_pose.y = tobe_optimized_[1];
    cur_pose.z = tobe_optimized_[2];

    // 目前就只看水平的偏移
    if (hypot(cur_pose.x - target_center_.x, cur_pose.y - target_center_.y) > target_update_dist_)
    {
      if (false == extractSurroundKeyFrames(cur_pose))
      {
        RCLCPP_WARN(this->get_logger(), "failed extract surround keyframes");
        continue;
      }
    }

    // scan matching refinement
    int converge_cnt = 0;
    for (int iter_cnt = 0; iter_cnt < max_opt_iters_; ++iter_cnt)
    {
      TicToc t_data, t_opt;
      ceres::Problem problem;
      ceres::LossFunction *loss_function = new ceres::HuberLoss(huber_s_);
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = max_iters_;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
      options.function_tolerance = func_tolerance_;
      options.gradient_tolerance = gradient_tolerance_;
      options.parameter_tolerance = param_tolerance_;
      problem.AddParameterBlock(tobe_optimized_, 6);

      int corner_correspondance = 0, surf_correspondance = 0;
      Eigen::Matrix4d m_tobe;
      m_tobe.block<3, 3>(0, 0) = (Eigen::AngleAxisd(tobe_optimized_[5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(tobe_optimized_[4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(tobe_optimized_[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
      m_tobe(0, 3) = tobe_optimized_[0];
      m_tobe(1, 3) = tobe_optimized_[1];
      m_tobe(2, 3) = tobe_optimized_[2];
      Eigen::Affine3d a_tobe(m_tobe);
      for (int i = 0; i < corner_ds->size(); ++i)
      {
        PointT point_sel = pcl::transformPoint<PointT, double>(corner_ds->points[i], a_tobe);
        kdtree_corner_target_->nearestKSearch(point_sel, 5, point_search_idx_, point_search_dist_);
        if (point_search_dist_[4] < 1.0)
        {
          std::vector<Eigen::Vector3d> near_corners;
          Eigen::Vector3d center(0., 0., 0.);
          for (int j = 0; j < 5; ++j)
          {
            Eigen::Vector3d tmp(pc_corner_target_ds_->points[point_search_idx_[j]].x, pc_corner_target_ds_->points[point_search_idx_[j]].y, pc_corner_target_ds_->points[point_search_idx_[j]].z);
            center += tmp;
            near_corners.push_back(tmp);
          }
          center = center / 5.;
          Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
          for (int j = 0; j < 5; ++j)
          {
            Eigen::Matrix<double, 3, 1> tmp = near_corners[j] - center;
            cov_mat = cov_mat + tmp * tmp.transpose();
          }
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);
          Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
          Eigen::Vector3d cp(corner_ds->points[i].x, corner_ds->points[i].y, corner_ds->points[i].z);
          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
          {
            Eigen::Vector3d lpj = 0.1 * unit_direction + center;
            Eigen::Vector3d lpl = -0.1 * unit_direction + center;
            problem.AddResidualBlock(new LidarEdgeCostFunction(cp, lpj, lpl), loss_function, tobe_optimized_);
            ++corner_correspondance;
          }
        }
      }

      for (int i = 0; i < surf_ds->size(); ++i)
      {
        PointT point_sel = pcl::transformPoint<PointT, double>(surf_ds->points[i], a_tobe);
        kdtree_surf_target_->nearestKSearch(point_sel, 5, point_search_idx_, point_search_dist_);
        if (point_search_dist_[4] < 1.0)
        {
          Eigen::Matrix<double, 5, 3> mat_a0;
          Eigen::Matrix<double, 5, 1> mat_b0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
          for (int j = 0; j < 5; ++j)
          {
            mat_a0(j, 0) = pc_surf_target_ds_->points[point_search_idx_[j]].x;
            mat_a0(j, 1) = pc_surf_target_ds_->points[point_search_idx_[j]].y;
            mat_a0(j, 2) = pc_surf_target_ds_->points[point_search_idx_[j]].z;
          }
          Eigen::Vector3d norm = mat_a0.colPivHouseholderQr().solve(mat_b0);
          double negative_OA_dot_norm = 1 / norm.norm();
          norm.normalize();
          bool plane_valid = true;
          for (int j = 0; j < 5; ++j)
          {
            if (std::fabs(norm(0) * pc_surf_target_ds_->points[point_search_idx_[j]].x +
                          norm(1) * pc_surf_target_ds_->points[point_search_idx_[j]].y +
                          norm(2) * pc_surf_target_ds_->points[point_search_idx_[j]].z + negative_OA_dot_norm) > 0.2)
            {
              plane_valid = false;
              break;
            }
          }
          if (plane_valid)
          {
            Eigen::Vector3d cp(surf_ds->points[i].x, surf_ds->points[i].y, surf_ds->points[i].z);
            problem.AddResidualBlock(new LidarPlaneCostFunction(cp, norm, negative_OA_dot_norm),
                                     loss_function, tobe_optimized_);
            ++surf_correspondance;
          }
        }
      }

      RCLCPP_INFO(this->get_logger(), "data association time: %.3fms", t_data.toc());
      RCLCPP_INFO(this->get_logger(), "corner correspondance: %d, surf correspondance: %d", corner_correspondance, surf_correspondance);

      TicToc t_solver;

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      RCLCPP_INFO(this->get_logger(), "localization solver time: %.3fms", t_solver.toc());
      std::cout << summary.BriefReport() << std::endl;

      if (summary.termination_type == ceres::TerminationType::CONVERGENCE)
      {
        ++converge_cnt;
      }
      // transformUpdate
      m2l_0_backup.block<3, 3>(0, 0) = (Eigen::AngleAxisd(tobe_optimized_[5], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(tobe_optimized_[4], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(tobe_optimized_[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
      m2l_0_backup(0, 3) = tobe_optimized_[0];
      m2l_0_backup(1, 3) = tobe_optimized_[1];
      m2l_0_backup(2, 3) = tobe_optimized_[2];

      tf_m2o_update_ = m2l_0_backup * o2l_o_backup.inverse();
      std::cout << "iter: " << iter_cnt << " opt time: " << t_opt.toc() << "ms" << std::endl;
    }

    float ratio = 1.0 * converge_cnt / max_opt_iters_;
    if (lol_confidence_ < 60. && ratio > 0.5)
    {
      lol_confidence_ = 60.;
    }
    else if (lol_confidence_ >= 60.)
    {
      if (ratio > 0.5)
      {
        lol_confidence_ += (100. - lol_confidence_) * (ratio - 0.5);
      }
      else
      {
        lol_confidence_ -= 40 * (ratio - 0.5);
      }
    }
    RCLCPP_INFO(this->get_logger(), "overall optimize time: %.3fms", t_all.toc());
  }
}

} // namespace localization