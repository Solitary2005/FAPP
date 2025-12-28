#ifndef _mot_mapping_H_
#define _mot_mapping_H_

#include <chrono>
#include <queue>
// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <obj_state_msgs/State.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
// Algorithm
#include "DBSCAN_kdtree.h"
#include "Hungarian.h"
#include "target_ekf.hpp"
#include "ekf_add_update.hpp"
#include "ikd_Tree.h"
#include "ikd_Tree_impl.h"

// save results
#include <unordered_map>
#include <fstream>
#include <iomanip>

using namespace std;
// 整个快速自适应感知和对动态物体的跟踪的框架
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Points;
typedef pcl::PointCloud<PointType>::Ptr PointsPtr;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;

struct MapParam {
  // DBSCAN Params
  int core_pts;
  double tolerance;
  int min_cluster;
  int max_cluster;
  // Dynamic Detection Params
  double thresh_dist;
  double thresh_var;
  // Lidar Params
  Eigen::Vector3d Range;
  // ROS Params
  std::string odom_topic;
  std::string lidar_topic;
};

struct MapData {
  Eigen::Matrix3d R; //Currnet IMU Orientation
  Eigen::Vector3d T; //Currnet IMU Positon
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ObjectState {
  int id; 
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d size;
};

struct VelEval {
  ros::Time first_seen;
  bool first_seen_set = false;
  bool converged_set = false;
  ros::Time converged_time;
  double sum_abs_err = 0.0;
  double sum_rel_err = 0.0;
  int samples = 0;
};

namespace mot_mapping {

class MappingRos {

private:
  // 用来点云下采样的
  pcl::VoxelGrid<PointType> vox;
  DBSCANKdtreeCluster<PointType> dbscan;
  KD_TREE<PointType>::Ptr ikdtree_ptr;
  // 前一帧的点云，初步下采样（0.1）
  PointsPtr Remaining_Points;
  int frame_num;
  // 聚类之前的点云，初步下采样（0.1）
  std::deque<PointsPtr> previous_points;
  std::deque<PointsPtr> buffer;
  std::deque<PointsPtr> input_point;
  std::vector<Eigen::Vector3d> previous_p;
  std::vector<Eigen::Vector3d> previous_v;
  std::vector<pcl::PointIndices> deleted_indices;
  std::vector<BoxPointType> delete_boxes;

  // std::vector<std::shared_ptr<Ekf>> trackers; //只给动态物体设定ekf
  std::vector<std::shared_ptr<EkfNew>> trackers; //只给动态物体设定ekf
  std::vector<ObjectState> detections;

  ros::Time t_start;

private:
  ros::NodeHandle &nh;
  ros::Timer ekf_predict_timer_, map_pub_timer_;
  // cloudPub发布动态点云，mapPub发布整体地图（经过2次降采样用于聚类的，也是用于聚类的点云的0.05下采样），staticMapPub发布静态地图
  ros::Publisher cloudPub, mapPub, staticMapPub, edgePub;
  ros::Publisher objectPosePub, statesPub;
  int id;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
  typedef boost::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

  SynchronizerCloudOdom sync_cloud_odom_;

  unique_ptr<MapData> md_;
  unique_ptr<MapParam> mp_;


  std::unordered_map<int, VelEval> vel_eval_;
  double vel_rel_threshold_{0.1};
  double vel_min_for_rel_{0.2};
  std::string metrics_path_{"~/workspace/FAPP/results/velocity_metrics.csv"};
  bool metrics_header_written_{false};


  float calc_dist(PointType p1, PointType p2);
  void generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths);
  void pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w);
  void pointsWorldToBody(const PointsPtr p_w, PointsPtr p_b, Eigen::Vector3d T);

  void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const nav_msgs::OdometryConstPtr& odom);
  void ekfPredictCallback(const ros::TimerEvent& e);
  void mapPubCallback(const ros::TimerEvent& e);
  void visualizeFunction(const std::vector<std::pair<int, int>> pairs);

  double iou(ObjectState state1, ObjectState state2);

  void recordVelocityEval(int tracker_id,
                        const Eigen::Vector3d &v_true,
                        const Eigen::Vector3d &v_est) {
  auto &M = vel_eval_[tracker_id];
  if (!M.first_seen_set) { M.first_seen = ros::Time::now(); M.first_seen_set = true; }
  double abs_err = (v_est - v_true).norm();
  double denom = std::max(v_true.norm(), vel_min_for_rel_);
  double rel_err = abs_err / denom;
  M.sum_abs_err += abs_err;
  M.sum_rel_err += rel_err;
  M.samples++;
  if (!M.converged_set && rel_err <= vel_rel_threshold_) {
    M.converged_set = true;
    M.converged_time = ros::Time::now();
    }
  }
  void flushVelocityEval(int tracker_id, const VelEval &M) {
  std::ofstream ofs(metrics_path_, std::ios::app);
  if (!ofs.is_open()) {
    ROS_WARN("Cannot open metrics path: %s", metrics_path_.c_str());
    return;
  }
  if (!metrics_header_written_) {
    ofs << "tracker_id,first_seen,converged_time,delta_sec,samples,avg_abs_err,avg_rel_err\n";
    metrics_header_written_ = true;
  }
  double delta = M.converged_set ? (M.converged_time - M.first_seen).toSec() : -1.0;
  double avg_abs = M.samples ? M.sum_abs_err / M.samples : 0.0;
  double avg_rel = M.samples ? M.sum_rel_err / M.samples : 0.0;
  ofs << tracker_id << ","
      << std::fixed << std::setprecision(6)
      << M.first_seen.toSec() << ","
      << (M.converged_set ? M.converged_time.toSec() : -1.0) << ","
      << delta << ","
      << M.samples << ","
      << avg_abs << ","
      << avg_rel << "\n";
  }
  

public:

  MappingRos(ros::NodeHandle &nh);
  ~MappingRos();
  
  void init();

};

}

#endif