#include "mapping_manager.h"

namespace mot_mapping {

MappingRos::MappingRos(ros::NodeHandle &nh):nh(nh) {}

MappingRos::~MappingRos() {
  for (const auto &kv : vel_eval_) flushVelocityEval(kv.first, kv.second);
  std::cout << "Exit Tracker" << std::endl;
}

void MappingRos::init() {
  t_start = ros::Time::now();
  frame_num = 1;
  id = 0;

  Remaining_Points.reset(new Points);
  ikdtree_ptr.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));

  mp_ = std::make_unique<MapParam>();
  md_ = std::make_unique<MapData>();
  // R：Eigen::Matrix3d，当前IMU朝向（旋转矩阵）。
  // T：Eigen::Vector3d，当前IMU位置（平移向量）。
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW：用于Eigen对齐

  nh.param("dbscan/core_pts", mp_->core_pts, 4);
  nh.param("dbscan/tolerance", mp_->tolerance, 0.02);
  nh.param("dbscan/min_cluster", mp_->min_cluster, 20);
  nh.param("dbscan/max_cluster", mp_->max_cluster, 800);
  nh.param("detection/thresh_dist", mp_->thresh_dist, 0.08);
  nh.param("detection/thresh_var", mp_->thresh_var, 0.28);
  nh.param("lidar/range_x", mp_->Range[0], 15.0);
  nh.param("lidar/range_y", mp_->Range[1], 15.0);
  nh.param("lidar/range_z", mp_->Range[2], 1.0);
  nh.param("ros/odom_topic", mp_->odom_topic, string("/odom"));
  nh.param("ros/lidar_topic", mp_->lidar_topic, string("/livox/lidar"));
  nh.param("eval/vel_rel_threshold", vel_rel_threshold_, 0.1);
  nh.param("eval/vel_min_for_rel", vel_min_for_rel_, 0.2);
  nh.param("eval/metrics_path", metrics_path_, std::string("/home/zwj/workspace/FAPP/results/velocity_metrics.csv"));
  // 创建ROS消息
  cloudPub = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_points", 10);
  mapPub = nh.advertise<sensor_msgs::PointCloud2>("/map_ros", 10);
  staticMapPub = nh.advertise<sensor_msgs::PointCloud2>("/static_map", 10);
  edgePub = nh.advertise<visualization_msgs::MarkerArray>("/box_edge", 1000);
  objectPosePub = nh.advertise<visualization_msgs::MarkerArray>("/object_pose", 10);
  statesPub = nh.advertise<obj_state_msgs::ObjectsStates>("/states", 10);

  std::cout << "INIT!" << std::endl;
  // 初始化消息订阅和同步
  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/map_generator/obj_cloud", 50));
  odom_sub_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/drone_0_visual_slam/odom", 25));
  // 将点云和里程计消息同步
  sync_cloud_odom_.reset(new message_filters::Synchronizer<MappingRos::SyncPolicyCloudOdom>(
      MappingRos::SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
  sync_cloud_odom_->registerCallback(boost::bind(&MappingRos::cloudOdomCallback, this, _1, _2));
  // 每隔0.02秒发布一次预测结果（由Kalman滤波器），每隔0.05秒发布一次地图

  ekf_predict_timer_ = nh.createTimer(ros::Duration(0.02), &MappingRos::ekfPredictCallback, this);
  map_pub_timer_ = nh.createTimer(ros::Duration(0.05), &MappingRos::mapPubCallback, this);
}

// 计算2个三维点的欧式距离
float MappingRos::calc_dist(PointType p1, PointType p2) {
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

void MappingRos::generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths) {
    float &x_dist = box_lengths[0];
    float &y_dist = box_lengths[1];
    float &z_dist = box_lengths[2];

    boxpoint.vertex_min[0] = center_pt.x - x_dist;
    boxpoint.vertex_max[0] = center_pt.x + x_dist;
    boxpoint.vertex_min[1] = center_pt.y - y_dist;
    boxpoint.vertex_max[1] = center_pt.y + y_dist;
    boxpoint.vertex_min[2] = center_pt.z - z_dist;
    boxpoint.vertex_max[2] = center_pt.z + z_dist;
}
// 提取在范围内的点，一个赋值的操作，没有进行坐标系转换，感觉可能跟里程计数据的输入有关
void MappingRos::pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w) {
  int num = p_b->points.size();
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    p << p_b->points[i].x, 
         p_b->points[i].y, 
         p_b->points[i].z;
    // p = md_->R * p + md_->T;
    if ((p - md_->T).head(2).norm() < 5.0) {
        PointType pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];

        p_w->points.push_back(pt);
    }
  }
}

void MappingRos::pointsWorldToBody(const PointsPtr p_w, PointsPtr p_b, Eigen::Vector3d T) {
  int num = p_w->points.size();
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    p << p_w->points[i].x, 
         p_w->points[i].y, 
         p_w->points[i].z;
    p = (p - md_->T);

    PointType pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    p_b->points.push_back(pt);
  }
}


void MappingRos::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg, 
                                   const nav_msgs::OdometryConstPtr& odom) {
                                    // odom是视觉里程计的话题（就是视觉里程计观测得到的数据）
  PointsPtr latest_cloud(new Points);
  pcl::fromROSMsg(*msg, *latest_cloud);

  md_->T << odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z;
  // w,x,y,z     
  md_->R = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                              odom->pose.pose.orientation.y, odom->pose.pose.orientation.z).toRotationMatrix();

  std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
  double compTime;
  // clear screen
  printf("\033[2J");
  printf("\033[1;1H");
  
  // Pointcloud Filter
  PointsPtr cloud_filtered(new Points);
  // 使用体素滤波进行下采样，让点数减少
  vox.setInputCloud(latest_cloud);
  vox.setLeafSize(0.1, 0.1, 0.1); //设置体素方格的大小
  vox.filter(*cloud_filtered);

  PointsPtr cloud_world(new Points); // 当前帧的点云（初步采样）
  PointsPtr nonfilter_pts(new Points); // 当前帧的原始点云（直接从雷达得到）
  PointsPtr PointToAdd(new Points); // 记录当前批的新增点
  PointsPtr ClusterPoints(new Points); // 用于聚类的点云（实际上是当前帧+缓冲池部分，这里buffer size是1 所以是当前帧+上一帧）
  pointsBodyToWorld(cloud_filtered, cloud_world);
  pointsBodyToWorld(latest_cloud, nonfilter_pts); //这里感觉如果不需要坐标系转换的话，可以直接用latest_cloud吧
  // 只存最近1帧的点云（下采样之前的）， buffer size = 1
  buffer.push_back(nonfilter_pts);
  if (buffer.size() > 1) {
    buffer.pop_front();
  }
  // 将buffer中的点云合并到ClusterPoints中
  for (int i = 0; i < buffer.size(); ++i) {
    *ClusterPoints += *buffer[i];
  }
  // 对聚类点云进行体素滤波，进一步减少点数
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(0.05, 0.05, 0.05);
  vox.filter(*ClusterPoints);

  Remaining_Points = ClusterPoints;
  
  PointsPtr All_Points(new Points);

  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Filter Time Cost (ms)： " << compTime <<std::endl;
  tic = std::chrono::high_resolution_clock::now();

  std::cout << "Input Size:" << ClusterPoints->size() << std::endl;
  
  sensor_msgs::PointCloud2 map_ros;
  PointsPtr OutputPoints(new Points);
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(0.05, 0.05, 0.05);
  vox.filter(*OutputPoints);

  pcl::toROSMsg(*OutputPoints, map_ros);
  map_ros.header.frame_id = "world";
  mapPub.publish(map_ros);

  // DBSCAN Cluster
  std::vector<pcl::PointIndices> cluster_indices;
  dbscan.setCorePointMinPts(mp_->core_pts);
  dbscan.setClusterTolerance(mp_->tolerance);
  dbscan.setMinClusterSize(mp_->min_cluster);
  dbscan.setMaxClusterSize(mp_->max_cluster);
  dbscan.setInputCloud(ClusterPoints);
  dbscan.setSearchMethod();
  dbscan.extractNano(cluster_indices); //领域搜索

  std::cout << "Cluster size: " << cluster_indices.size() << std::endl;
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Cluster Time Cost (ms)： " << compTime <<std::endl;
  tic = std::chrono::high_resolution_clock::now();
  // 使用ikd-tree进行F帧的地图管理
  //第1帧只缓存并返回；
  if (frame_num < 2){
    previous_points.push_back(cloud_world);
    frame_num++;
    return;
  }

  int pt_size = previous_points[0]->points.size();
  //ikd-tree
  if (ikdtree_ptr->Root_Node == nullptr) {
    
    //第2帧先用第1帧构树，再把缓存更新为第2帧；
    ikdtree_ptr->Build(previous_points[0]->points); 
    previous_points.push_back(cloud_world); // 
    previous_points.pop_front();
  }
  else {
    // 第3帧开始进行增量添加
    PointVector PointNoNeedDownsample;
    bool need_add = true;

    for (int i = 0; i < pt_size; ++i) {
      /*
      ikd-tree的插入过程
      首先将整个空间体素化，并明确新点落入哪个体素（目标体素）；
      然后向ikd-Tree查询目标体素内是否已经有点以及有哪些点（查询过程参考box-wise delete）；
      如果已经有点了，将已有点和新点一起排序，找出离体素中心最近的那个点，然后做判断：
      如果最近的点是已有相似的点，意味着新点无必要再插入了，结束处理；
      如果最近的点是新点，则把已有点全部标记删除，并插入新点；
      如果体素内尚不存在点，则直接插入新点。
      */
      PointType mid_point; 
      mid_point.x = floor(previous_points[0]->points[i].x/0.1)*0.1 + 0.5 * 0.1;
      mid_point.y = floor(previous_points[0]->points[i].y/0.1)*0.1 + 0.5 * 0.1;
      mid_point.z = floor(previous_points[0]->points[i].z/0.1)*0.1 + 0.5 * 0.1;
      // float dist  = calc_dist(cloud_filtered->points[i],mid_point);
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      ikdtree_ptr->Nearest_Search(mid_point, 1, points_near, pointSearchSqDis);
      // ##################################################
      // 修复一下：如果近邻为空则直接加入，免得访问points_near越界了
      if (points_near.empty()) {
        PointNoNeedDownsample.push_back(previous_points[0]->points[i]);
        PointToAdd->points.push_back(previous_points[0]->points[i]);
        continue;
      }
      // ##################################################
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * 0.1 || 
          fabs(points_near[0].y - mid_point.y) > 0.5 * 0.1 || 
          fabs(points_near[0].z - mid_point.z) > 0.5 * 0.1) {
        PointNoNeedDownsample.push_back(previous_points[0]->points[i]); 
        PointToAdd->points.push_back(previous_points[0]->points[i]); // 动态改变的，包含这一批所有新增点
      }
    }
    // int add_point_size = ikdtree_ptr->Add_Points(PointToAdd, true);
    ikdtree_ptr->Add_Points(PointNoNeedDownsample, false); 
    input_point.push_back(PointToAdd); // 保存新增点的队列，每个元素是一批点
    if (input_point.size() > 25) {
      PointVector PointDelete;
      for (auto& delet_pt: input_point[0]->points) {
        PointDelete.push_back(delet_pt);
      }
      ikdtree_ptr->Delete_Points(PointDelete);
      input_point.pop_front();
    } // 动态删除ikd-tree的旧点，保持滑动窗大小为25帧
    // 用于存上一帧点云
    previous_points.push_back(cloud_world);
    previous_points.pop_front();
  }
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "ikd-Tree Time Cost (ms)： " << compTime << std::endl;
  tic = std::chrono::high_resolution_clock::now();
  std::cout << "ikd-Tree Size: " << ikdtree_ptr->size() << std::endl;
  // 可以导出当前ikd-tree中的所有点，用ROS发布，可以用于调试/可视化
  // ikdtree_ptr->flatten(ikdtree_ptr->Root_Node, ikdtree_ptr->PCL_Storage, NOT_RECORD);
  // All_Points->points = ikdtree_ptr->PCL_Storage;
  // All_Points->width = All_Points->points.size();
  // All_Points->height = 1;
  // All_Points->is_dense = true;

  // sensor_msgs::PointCloud2 previous_cloud;
  // pcl::toROSMsg(*All_Points, previous_cloud);
  // previous_cloud.header.frame_id = "world";
  // cloudPub.publish(previous_cloud);


  // 开始划分动态点和静态点
  PointsPtr dynamic_points(new Points);

  int k = 0;
  detections.clear();
  deleted_indices.clear();
  // ###########################################
  if (ikdtree_ptr->size() == 0) {
    // 空树情况下跳过本轮判别，等待下一帧构树或增量插入完成
    sensor_msgs::PointCloud2 dynamic_pts;
    pcl::toROSMsg(*dynamic_points, dynamic_pts);
    dynamic_pts.header.frame_id = "world";
    cloudPub.publish(dynamic_pts);
    return;
  }
  // #############################################
  for (auto& getIndices: cluster_indices) {
    // 每一类
    PointsPtr cluster(new Points);
    PointVector PointDelete;
    double avg_dist = 0; //累加最近邻距离
    Eigen::Vector3d cluster_center;
    cluster_center.setZero();
    std::vector<double> avg_buffer;//当前地图中到该类中每点的最近点
    for (auto& index : getIndices.indices) {
      // 每一个点
      cluster->points.push_back(ClusterPoints->points[index]);
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      // 只找最近的3个点
      ikdtree_ptr->Nearest_Search(ClusterPoints->points[index], 3, points_near, pointSearchSqDis);
      avg_dist += sqrt(pointSearchSqDis[0]);
      avg_buffer.push_back(sqrt(pointSearchSqDis[0]));
      cluster_center += Eigen::Vector3d(ClusterPoints->points[index].x, ClusterPoints->points[index].y, ClusterPoints->points[index].z);
    }
    // 由于作者判断静态/动态点的依据是
    // 对于一个持续移动的物体，其点云与之前帧点云集之间存在较大距离，同时距离分布相对均匀。
    // 所以需要根据每个类中点到ikd-tree中点的平均距离和方差来判断
    // 得到该类的平均距离和方差
    avg_dist = avg_dist/getIndices.indices.size();
    cluster_center = cluster_center/getIndices.indices.size();
    double var_dist = 0;
    for (auto& dist : avg_buffer) {
      var_dist += (dist - avg_dist) * (dist - avg_dist) / (avg_dist * avg_dist);
    }
    var_dist = var_dist/getIndices.indices.size();
    // 判断该簇是否正在被一个EKF tracker跟踪
    bool on_track = false;
    for (int i = 0; i < trackers.size(); ++i) {
      Eigen::Vector3d tracker_center = trackers[i]->pos();
      double dist = sqrt((cluster_center(0) - tracker_center(0)) * (cluster_center(0) - tracker_center(0)) + 
                         (cluster_center(1) - tracker_center(1)) * (cluster_center(1) - tracker_center(1)) + 
                         (cluster_center(2) - tracker_center(2)) * (cluster_center(2) - tracker_center(2)));
      // 这块判断ekf跟踪的不懂？？？
      if (dist < 0.5 && trackers[i]->age > 3 && trackers[i]->vel().norm() > 0.2) {
        on_track = true;
        break;
      }
    }
    // ######################### 不懂用来干啥
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    pcl::PointXYZ minPt, maxPt;
	  pcl::getMinMax3D(*cluster, minPt, maxPt);

    PointType center_pt;
    center_pt.x = (maxPt.x + minPt.x)/2;
    center_pt.y = (maxPt.y + minPt.y)/2;
    center_pt.z = (maxPt.z + minPt.z)/2;

    double radius_xy = (center_pt.x - md_->T[0]) * (center_pt.x - md_->T[0]) + 
                       (center_pt.y - md_->T[1]) * (center_pt.y - md_->T[1]);
    double radius_z = (center_pt.z - md_->T[2]) * (center_pt.z - md_->T[2]);
    // ######################### 不懂用来干啥
    // 平均距离大于阈值且小于5米，且距离方差小于阈值，或者该类正在被ekf跟踪，则认为是动态物体
    if (avg_dist > mp_->thresh_dist && avg_dist < 5.0 && var_dist < mp_->thresh_var || on_track) {
      // std::cout << "on_track: " << on_track << std::endl;
      // std::cout << "var_dist: " << var_dist << std::endl;
      // std::cout << "avg_dist: " << avg_dist << std::endl;
      ObjectState state;
      state.id = k;
      state.position = cluster_center;
      // 为什么要记一个size
      state.size << maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z;
      detections.push_back(state);
      deleted_indices.push_back(getIndices);
      for (auto& index : getIndices.indices) {
        dynamic_points->points.push_back(ClusterPoints->points[index]);
      }
      // BoxPointType box;
      // box.vertex_min[0] = minPt.x-0.2; box.vertex_min[1] = minPt.y-0.2; box.vertex_min[2] = minPt.z-0.2;
      // box.vertex_max[0] = maxPt.x+0.2; box.vertex_max[1] = maxPt.y+0.2; box.vertex_max[2] = maxPt.z+0.2;
      // delete_boxes.push_back(box);
    }
  }
  // 结束动态点提取

  sensor_msgs::PointCloud2 dynamic_pts;
  pcl::toROSMsg(*dynamic_points, dynamic_pts);
  dynamic_pts.header.frame_id = "world";
  cloudPub.publish(dynamic_pts);

  // Tracking & EKF
  // 初始化跟踪器
  // if(trackers.size() == 0) {
  //   for(int i = 0; i < detections.size(); ++i) {
  //     std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
  //     ekfPtr->reset(detections[i].position, id);
  //     id++;
  //     trackers.push_back(ekfPtr);
  //     previous_p.push_back(detections[i].position);
  //     Eigen::Vector3d v0(0,0,0);
  //     previous_v.push_back(v0);
  //   }
  //   return;
  // }


  // obj_state_msgs::ObjectsStates states;
  // std::vector<std::pair<int, int>> matchedPairs;
  // // 为每个动态物体分配卡尔曼滤波跟踪器
  // for (int i = 0; i < detections.size(); ++i) {
  //   double min_dist = 1000000;
  //   int min_index = -1;
  //   Eigen::Vector3d det_pos = detections[i].position;
  //   // 更新跟踪器，并为动态物体找到最近的跟踪器
  //   for (int j = 0; j < trackers.size(); ++j) {
  //     trackers[j]->age = trackers[j]->age + 1;
  //     Eigen::Vector3d track_pos = trackers[j]->pos();
  //     // 这个不是欧式距离吗？为什么论文里写的是马式距离？马式距离和欧式距离的区别是什么？
  //     double dist = (det_pos - track_pos).norm();
  //     if (dist < min_dist) {
  //       min_dist = dist;
  //       min_index = j;
  //     }
  //   }

  //   if (min_dist < 0.8) {
  //     // 认为该跟踪器对应该动态物体
  //     // std::cout << "id:" << trackers[min_index]->id << std::endl;
  //     // 使用上一帧和当前帧的平均速度进行卡尔曼滤波的更新
  //     Eigen::Vector3d vel_detect = (detections[i].position - previous_p[min_index]) / 0.02;
  //     trackers[min_index]->update(detections[i].position, 0.5*vel_detect + 0.5*previous_v[min_index] );

  //     matchedPairs.push_back(std::make_pair(i, min_index));
  //     // 用ROS发布状态信息
  //     ObjectState state;
  //     state.position = trackers[min_index]->pos();
  //     state.velocity = trackers[min_index]->vel();
  //     obj_state_msgs::State statemsg;
  //     statemsg.header.stamp = ros::Time::now();
  //     statemsg.position.x = state.position[0]; statemsg.position.y = state.position[1]; statemsg.position.z = state.position[2];
  //     statemsg.velocity.x = state.velocity[0]; statemsg.velocity.y = state.velocity[1]; statemsg.velocity.z = state.velocity[2];
  //     statemsg.size.x = 0; statemsg.size.y = 0; statemsg.size.z = 0;
  //     states.states.push_back(statemsg);

  //   } else {
  //     // 为该动态物体新建一个跟踪器
  //     std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
  //     ekfPtr->reset(detections[i].position, id);
  //     id++;
  //     trackers.push_back(ekfPtr);
  //   }
  // }
  //使用更新的EKF#############################################
    if(trackers.size() == 0) {
    for(int i = 0; i < detections.size(); ++i) {
      std::shared_ptr<EkfNew> ekfPtr = std::make_shared<EkfNew>(0.02);
      ekfPtr->reset(detections[i].position, id);
      vel_eval_[ekfPtr->id].first_seen = ros::Time::now();
      vel_eval_[ekfPtr->id].first_seen_set = true;
      id++;
      trackers.push_back(ekfPtr);
      previous_p.push_back(detections[i].position);
      Eigen::Vector3d v0(0,0,0);
      previous_v.push_back(v0);
      
    }
    return;
  }


  obj_state_msgs::ObjectsStates states;
  std::vector<std::pair<int, int>> matchedPairs;
  // 为每个动态物体分配卡尔曼滤波跟踪器
  for (int i = 0; i < detections.size(); ++i) {
    double min_dist = 1000000;
    int min_index = -1;
    Eigen::Vector3d det_pos = detections[i].position;
    // 更新跟踪器，并为动态物体找到最近的跟踪器
    for (int j = 0; j < trackers.size(); ++j) {
      trackers[j]->age = trackers[j]->age + 1;
      Eigen::Vector3d track_pos = trackers[j]->pos();
      // 这个不是欧式距离吗？为什么论文里写的是马式距离？马式距离和欧式距离的区别是什么？
      double dist = (det_pos - track_pos).norm();
      if (dist < min_dist) {
        min_dist = dist;
        min_index = j;
      }
    }

    if (min_dist < 0.8) {
      // 认为该跟踪器对应该动态物体
      // std::cout << "id:" << trackers[min_index]->id << std::endl;
      // 使用上一帧和当前帧的平均速度进行卡尔曼滤波的更新
      Eigen::Vector3d vel_detect = (detections[i].position - previous_p[min_index]) / 0.02;
      trackers[min_index]->update(detections[i].position, 0.5*vel_detect + 0.5*previous_v[min_index] );
      recordVelocityEval(trackers[min_index]->id,
                   /* v_true */ vel_detect,
                   /* v_est  */ trackers[min_index]->vel());
      matchedPairs.push_back(std::make_pair(i, min_index));
      // 用ROS发布状态信息
      ObjectState state;
      state.position = trackers[min_index]->pos();
      state.velocity = trackers[min_index]->vel();
      obj_state_msgs::State statemsg;
      statemsg.header.stamp = ros::Time::now();
      statemsg.position.x = state.position[0]; statemsg.position.y = state.position[1]; statemsg.position.z = state.position[2];
      statemsg.velocity.x = state.velocity[0]; statemsg.velocity.y = state.velocity[1]; statemsg.velocity.z = state.velocity[2];
      statemsg.size.x = 0; statemsg.size.y = 0; statemsg.size.z = 0;
      states.states.push_back(statemsg);

    } else {
      // 为该动态物体新建一个跟踪器
      std::shared_ptr<EkfNew> ekfPtr = std::make_shared<EkfNew>(0.02);
      ekfPtr->reset(detections[i].position, id);
      vel_eval_[ekfPtr->id].first_seen = ros::Time::now();
      vel_eval_[ekfPtr->id].first_seen_set = true;
      id++;
      trackers.push_back(ekfPtr);
    }
  }
  // #######################################
  // 加了一句判断，只有跟踪器有输出时才发布状态信息
  // if (states.states.size() > 0)
  //   statesPub.publish(states);
  // #######################################
  statesPub.publish(states);
  // 可视化跟踪结果
  visualizeFunction(matchedPairs);
  // 删除长时间未被更新的跟踪器（age大于20的）
  for (auto it = trackers.begin(); it != trackers.end();) {
      // std::cout << "dt:" << (*it)->age - (*it)->update_num << std::endl;
      if ((*it)->age - (*it)->update_num > 20){
        int tid = (*it)->id;
        auto mit = vel_eval_.find(tid);
        if (mit != vel_eval_.end()) { 
          flushVelocityEval(tid, mit->second); 
          vel_eval_.erase(mit); }
          it = trackers.erase(it);
      }
      else 
        it++;
  }    
  // 使用previous_p和previous_v记录上一帧的位置信息和速度信息
  previous_p.clear();
  previous_v.clear();
  for (int i = 0; i < trackers.size(); ++i ) {
    previous_p.push_back(trackers[i]->pos());
    previous_v.push_back(trackers[i]->vel());
  }
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Tracking Time Cost (ms)： " << compTime <<std::endl;
}

void MappingRos::ekfPredictCallback(const ros::TimerEvent& e) {
  if (trackers.size() == 0)
    return;

  for (int i = 0; i < trackers.size(); ++i) {
    double update_dt = (ros::Time::now() - trackers[i]->last_update_stamp_).toSec();
    trackers[i]->predict();
  }
}

void MappingRos::mapPubCallback(const ros::TimerEvent& e) {
  if (Remaining_Points->points.size() == 0)
    return;
  
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // KD_TREE<PointType>::Ptr delete_tree;
  // delete_tree.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));
  // delete_tree->Build(Remaining_Points->points);
  // delete_tree->Delete_Point_Boxes(delete_boxes);

  // Points static_pts;
  // delete_tree->flatten(delete_tree->Root_Node, delete_tree->PCL_Storage, NOT_RECORD);
  // static_pts.points = delete_tree->PCL_Storage;
  // static_pts.width = static_pts.points.size();
  // static_pts.height = 1;
  // static_pts.is_dense = true;


  // delete_boxes.clear();
  // sensor_msgs::PointCloud2 map_static;
  // pcl::toROSMsg(static_pts, map_static);
  // map_static.header.frame_id = "world";
  // staticMapPub.publish(map_static);
}

// 论文里说的马式距离，但是好像没用？
double MappingRos::iou(ObjectState state1, ObjectState state2) {
  double distance = (state1.position - state2.position).norm();
  double iou = atan(distance)*2/M_PI;
  return iou;
}

void MappingRos::visualizeFunction(const std::vector<std::pair<int, int>> pairs) {
  visualization_msgs::MarkerArray poses;
  visualization_msgs::MarkerArray boxes;

  for (auto pair : pairs) {
    int detectionIndex = pair.first;
    int trackerIndex = pair.second;
    pcl::PointXYZ minPt, maxPt;
    // if (detections[detectionIndex].position[1] - detections[detectionIndex].size[1]/2 > 1.7 ||
    //     detections[detectionIndex].position[1] < -2.0 ||
    //     detections[detectionIndex].position[2] > 1.8)
    //   continue;
    minPt.x = detections[detectionIndex].position[0] - detections[detectionIndex].size[0]/2;
    minPt.y = detections[detectionIndex].position[1] - detections[detectionIndex].size[1]/2;
    minPt.z = detections[detectionIndex].position[2] - detections[detectionIndex].size[2]/2;
    maxPt.x = detections[detectionIndex].position[0] + detections[detectionIndex].size[0]/2;
    maxPt.y = detections[detectionIndex].position[1] + detections[detectionIndex].size[1]/2;
    maxPt.z = detections[detectionIndex].position[2] + detections[detectionIndex].size[2]/2;
    visualization_msgs::Marker edgeMarker;
    edgeMarker.id = detectionIndex;
    edgeMarker.header.stamp = ros::Time::now();
    edgeMarker.header.frame_id = "world";
    edgeMarker.pose.orientation.w = 1.00;
    edgeMarker.lifetime = ros::Duration(0.1);
    edgeMarker.type = visualization_msgs::Marker::LINE_STRIP;
    edgeMarker.action = visualization_msgs::Marker::ADD;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 1.00;
    edgeMarker.color.g = 0.50;
    edgeMarker.color.b = 0.00;
    edgeMarker.color.a = 0.80;
    edgeMarker.scale.x = 0.1;
    geometry_msgs::Point point[8];
    point[0].x = minPt.x; point[0].y = maxPt.y; point[0].z = maxPt.z;
    point[1].x = minPt.x; point[1].y = minPt.y; point[1].z = maxPt.z;
    point[2].x = minPt.x; point[2].y = minPt.y; point[2].z = minPt.z;
    point[3].x = minPt.x; point[3].y = maxPt.y; point[3].z = minPt.z;
    point[4].x = maxPt.x; point[4].y = maxPt.y; point[4].z = minPt.z;
    point[5].x = maxPt.x; point[5].y = minPt.y; point[5].z = minPt.z;
    point[6].x = maxPt.x; point[6].y = minPt.y; point[6].z = maxPt.z;
    point[7].x = maxPt.x; point[7].y = maxPt.y; point[7].z = maxPt.z;
    for (int l = 0; l < 8; l++) {
      edgeMarker.points.push_back(point[l]);
    }
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[3]);
    edgeMarker.points.push_back(point[2]);
    edgeMarker.points.push_back(point[5]);
    edgeMarker.points.push_back(point[6]);
    edgeMarker.points.push_back(point[1]);
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[7]);
    edgeMarker.points.push_back(point[4]);
    boxes.markers.push_back(edgeMarker); 

    visualization_msgs::Marker poseMarker;
    ObjectState state;
    state.position = trackers[trackerIndex]->pos();
    state.velocity = trackers[trackerIndex]->vel();
    poseMarker.id = trackerIndex+100;
    poseMarker.header.stamp = ros::Time::now();
    poseMarker.header.frame_id = "world";
    poseMarker.lifetime = ros::Duration(0.1);
    poseMarker.type = visualization_msgs::Marker::ARROW;
    poseMarker.action = visualization_msgs::Marker::ADD;
    poseMarker.ns = "objectpose";
    poseMarker.color.r = 0.00;
    poseMarker.color.g = 1.00;
    poseMarker.color.b = 0.00;
    poseMarker.color.a = 1.00;
    poseMarker.scale.x = 0.10;
    poseMarker.scale.y = 0.18;
    poseMarker.scale.z = 0.30;
    poseMarker.pose.orientation.w = 1.0;
    geometry_msgs::Point arrow[2];
    arrow[0].x = state.position[0]; arrow[0].y = state.position[1]; arrow[0].z = state.position[2];
    arrow[1].x = state.position[0] + 1.0*state.velocity[0]/state.velocity.norm(); 
    arrow[1].y = state.position[1] + 1.0*state.velocity[1]/state.velocity.norm(); 
    arrow[1].z = state.position[2] + 1.0*state.velocity[2]/state.velocity.norm();
    // std::cout << "vx:" << state.velocity[0] << std::endl;
    // std::cout << "vy:" << state.velocity[1] << std::endl;
    // std::cout << "vz:" << state.velocity[2] << std::endl;
    poseMarker.points.push_back(arrow[0]);
    poseMarker.points.push_back(arrow[1]);
    if (state.velocity.norm() > 0.01)
      poses.markers.push_back(poseMarker);
  }
  objectPosePub.publish(poses);
  edgePub.publish(boxes);

}

}
