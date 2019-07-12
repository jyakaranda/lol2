/**
 * @file lolLocalization.h
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-07-12
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef __LOL2_LOCALIZATION__
#define __LOL2_LOCALIZATION__

#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <deque>
#include <string>
#include <thread>
#include <vector>
#include <array>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/angles.h>

#include <ceres/ceres.h>

#include "alego2/utility.h"

namespace localization
{

using namespace ceres;

class LolLocalization : public rclcpp::Node
{
public:
  LolLocalization();
  ~LolLocalization() {}
  LolLocalization(const LolLocalization &l) = delete;

  void run(std::shared_ptr<LolLocalization> lol);

private:
  // 点云地图中 intensity 为 keypose 的 id
  using PointType = pcl::PointXYZI;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_corner_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_surf_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_outlier_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_lol_pose_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_corner_target_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf_target_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_corner_source_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf_source_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_test_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_confidence_;
  tf2_ros::TransformBroadcaster *tf_broad_;

  double tobe_optimized_[6];
  float lol_confidence_;

  pcl::PointCloud<PointType>::Ptr laser_corner_;
  pcl::PointCloud<PointType>::Ptr laser_surf_;
  pcl::PointCloud<PointType>::Ptr laser_outlier_;
  pcl::PointCloud<PointType>::Ptr laser_corner_ds_;
  pcl::PointCloud<PointType>::Ptr laser_surf_ds_;
  pcl::PointCloud<PointType>::Ptr laser_outlier_ds_;

  double time_laser_corner_, time_laser_surf_, time_laser_outlier_, time_laser_odom_;
  bool new_laser_corner_, new_laser_surf_, new_laser_outlier_, new_laser_odom_;

  // 一个 keypose 对应三种点云，点云在 vector 中的 index 由 keypose 在 keyposes_3d_ 中的 index 决定，点云已根据 keypose 转换到 global 坐标系中
  std::vector<pcl::PointCloud<PointType>::Ptr> corner_keyframes_;
  std::vector<pcl::PointCloud<PointType>::Ptr> surf_keyframes_;
  std::vector<pcl::PointCloud<PointType>::Ptr> outlier_keyframes_;
  pcl::PointCloud<PointType>::Ptr keyposes_3d_;

  // 局部 target corner, surf 点云，作为局部地图进行配准
  pcl::PointCloud<PointType>::Ptr pc_corner_target_;
  pcl::PointCloud<PointType>::Ptr pc_surf_target_;
  pcl::PointCloud<PointType>::Ptr pc_corner_target_ds_;
  pcl::PointCloud<PointType>::Ptr pc_surf_target_ds_;
  PointType target_center_;
  std::vector<int> surround_keyposes_id_;
  std::vector<pcl::PointCloud<PointType>::Ptr> surround_corner_keyframes_;
  std::vector<pcl::PointCloud<PointType>::Ptr> surround_surf_keyframes_;
  std::vector<pcl::PointCloud<PointType>::Ptr> surround_outlier_keyframes_;
  int batch_cnt_;
  PointCloudT::Ptr local_corner_;
  PointCloudT::Ptr local_surf_;
  PointCloudT::Ptr local_outlier_;

  Eigen::Matrix4d tf_o2l_0_;
  Eigen::Matrix4d tf_m2l_0_;

  std::vector<int> point_search_idx_;
  std::vector<float> point_search_dist_;

  // 快速查找 cur_pose 附近的 keypose，进而提取附近点云作为 target map
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_keyposes_3d_;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_target_;
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_target_;
  // params
  double surround_search_radius_;
  int surround_search_num_;

  // voxel filter
  pcl::VoxelGrid<PointType> ds_corner_;
  pcl::VoxelGrid<PointType> ds_surf_;
  pcl::VoxelGrid<PointType> ds_outlier_;
  pcl::VoxelGrid<PointType> ds_surround_keyposes_;

  // params
  double corner_leaf_, surf_leaf_, outlier_leaf_, surround_keyposes_leaf_;
  std::string fn_poses_, fn_corner_, fn_surf_, fn_outlier_;
  double target_update_dist_; // 与 target_center_ 偏移 target_update_dist_ 以上时更新局部 target map
  int batch_size_;
  int max_opt_iters_;
  double huber_s_;
  int max_iters_;
  double func_tolerance_;
  double gradient_tolerance_;
  double param_tolerance_;

  geometry_msgs::msg::PoseStamped cur_laser_pose_, pre_laser_pose_;
  std::deque<geometry_msgs::msg::PoseStamped> history_poses_;
  Eigen::Matrix4d tf_b2l_;
  Eigen::Matrix4d tf_m2o_;
  Eigen::Matrix4d tf_o2b_;
  Eigen::Matrix4d tf_m2o_update_;

  std::mutex mtx_;
  std::shared_ptr<LolLocalization> self_;
  std::thread opt_thread_;

  void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cornerCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void surfCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void outlierCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief 初始化工作，载入参数
   * 
   * @return true 
   * @return false 
   */
  bool init();

  /**
   * @brief 提取 p 附近的 corner, surf key frames 作为局部 target map 用于点云配准
   * 
   * @param p 
   */
  bool extractSurroundKeyFrames(const PointType &p);

  void optimizeThread();
};
} // namespace localization

#endif