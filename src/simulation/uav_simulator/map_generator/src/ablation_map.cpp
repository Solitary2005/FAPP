#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <geometry_msgs/TwistStamped.h>

// 一个沿x轴以恒定速度运动的立方体点云生成器，用于消融实验

int main(int argc, char** argv) {
  ros::init(argc, argv, "ablation_map");
  ros::NodeHandle nh("~");


  std::string cloud_topic = "/map_generator/obj_cloud";
  std::string frame_id = "world";
  double rate_hz = 50.0;          // 发布频率
  double speed = 5.0;             // m/s，沿 X 方向
  double size_x = 0.6, size_y = 0.8, size_z = 1.0; // 长方体三轴边长（米）
  int pts_x = 5, pts_y = 5, pts_z = 13;             // 三轴采样点数
  double start_x = -10.0, start_y = 0.0, start_z = 1.0; // 初始位置
  double map_x_size = 35.0, map_y_size = 35.0, map_z_size = 5.0; // 地图边界，用于重置

  std::string vel_topic = "/map_generator/obj_velocity";
  nh.param("vel_topic", vel_topic, vel_topic);
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(vel_topic, 1);

  nh.param("cloud_topic", cloud_topic, cloud_topic);
  nh.param("frame_id", frame_id, frame_id);
  nh.param("rate_hz", rate_hz, rate_hz);
  nh.param("speed", speed, speed);
  nh.param("size_x", size_x, size_x);
  nh.param("size_y", size_y, size_y);
  nh.param("size_z", size_z, size_z);
  nh.param("pts_x", pts_x, pts_x);
  nh.param("pts_y", pts_y, pts_y);
  nh.param("pts_z", pts_z, pts_z);
  nh.param("start_x", start_x, start_x);
  nh.param("start_y", start_y, start_y);
  nh.param("start_z", start_z, start_z);
  nh.param("map/x_size", map_x_size, map_x_size);
  nh.param("map/y_size", map_y_size, map_y_size);
  nh.param("map/z_size", map_z_size, map_z_size);

  ros::Publisher obj_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1);
  ros::Rate rate(rate_hz);

  const double dt = 1.0 / rate_hz;
  double x = start_x;
  double y = start_y;
  double z = start_z;

  // 地图边界（中心在原点，按 size/2 作为墙位置）
  const double x_min = -map_x_size / 2.0;
  const double x_max =  map_x_size / 2.0;
  const double y_min = -map_y_size / 2.0;
  const double y_max =  map_y_size / 2.0;
  const double z_min = 0.0;
  const double z_max =  map_z_size;

  // 立方体内部点采样步长
  const double half_x = size_x / 2.0;
  const double half_y = size_y / 2.0;
  const double half_z = size_z / 2.0;

  const double step_x = (pts_x > 1) ? (size_x / (pts_x - 1)) : size_x;
  const double step_y = (pts_y > 1) ? (size_y / (pts_y - 1)) : size_y;
  const double step_z = (pts_z > 1) ? (size_z / (pts_z - 1)) : size_z;

    

  while (ros::ok()) {
    // 越界检查
    if (x - half_x < x_min || x + half_x > x_max ||
        y - half_y < y_min || y + half_y > y_max ||
        z - half_z < z_min || z + half_z > z_max) {
    x = start_x; y = start_y; z = start_z;
    }

    // 构造立方体点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->points.reserve(pts_x * pts_y * pts_z);
    for (int ix = 0; ix < pts_x; ++ix) {
        for (int iy = 0; iy < pts_y; ++iy) {
            for (int iz = 0; iz < pts_z; ++iz) {
            pcl::PointXYZ p;
            p.x = x + (-half_x + ix * step_x);
            p.y = y + (-half_y + iy * step_y);
            p.z = z + (-half_z + iz * step_z);
            cloud->points.push_back(p);
            }
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // 发布
    const ros::Time now = ros::Time::now();
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = now;
    msg.header.frame_id = frame_id;
    obj_pub.publish(msg);

    geometry_msgs::TwistStamped vel;
    vel.header.stamp = now;
    vel.header.frame_id = frame_id;
    vel.twist.linear.x = 5.0;
    vel.twist.linear.y = 0.0;
    vel.twist.linear.z = 0.0;
    vel.twist.angular.x = 0.0;
    vel.twist.angular.y = 0.0;
    vel.twist.angular.z = 0.0;
    vel_pub.publish(vel);

    // 匀速运动（沿 X 轴）
    x += speed * dt;

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

// 一个运动方向快速变化的物体，其加速度在0至1.0秒间为3米/秒²，
// 1.0至1.2秒间骤降至-30米/秒²，随后在1.2至2.0秒间回升至3米/秒²
// int main(int argc, char** argv) {
//   ros::init(argc, argv, "ablation_map");
//   ros::NodeHandle nh("~");

//   std::string cloud_topic = "/map_generator/obj_cloud";
//   std::string frame_id = "world";
  
//   double rate_hz = 50.0;          // 发布频率
//   double speed = 0.0;             // 初始速度（m/s），沿 X 方向；保留旧参数名
//   double init_speed = speed;      // 可用 init_speed 参数覆盖
//   double size_x = 0.6, size_y = 0.8, size_z = 1.0; // 长方体三轴边长（米）
//   int pts_x = 5, pts_y = 5, pts_z = 13;            // 三轴采样点数
//   double start_x = 10.0, start_y = 0.0, start_z = 1.0; // 初始位置
//   double map_x_size = 35.0, map_y_size = 35.0, map_z_size = 5.0; // 地图边界，用于重置

//   std::string vel_topic = "/map_generator/obj_velocity";
//   nh.param("vel_topic", vel_topic, vel_topic);
//   ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(vel_topic, 1);

//   nh.param("cloud_topic", cloud_topic, cloud_topic);
//   nh.param("frame_id", frame_id, frame_id);
//   nh.param("rate_hz", rate_hz, rate_hz);
//   nh.param("speed", speed, speed);                 // 兼容旧参数
//   nh.param("init_speed", init_speed, speed);       // 若提供 init_speed，则覆盖
//   nh.param("size_x", size_x, size_x);
//   nh.param("size_y", size_y, size_y);
//   nh.param("size_z", size_z, size_z);
//   nh.param("pts_x", pts_x, pts_x);
//   nh.param("pts_y", pts_y, pts_y);
//   nh.param("pts_z", pts_z, pts_z);
//   nh.param("start_x", start_x, start_x);
//   nh.param("start_y", start_y, start_y);
//   nh.param("start_z", start_z, start_z);
//   nh.param("map/x_size", map_x_size, map_x_size);
//   nh.param("map/y_size", map_y_size, map_y_size);
//   nh.param("map/z_size", map_z_size, map_z_size);

//   ros::Publisher obj_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1);
//   ros::Rate rate(rate_hz);

//   const double dt = 1.0 / rate_hz;
//   double x = start_x;
//   double y = start_y;
//   double z = start_z;
//   double vx = init_speed; // X 轴速度

//   // 地图边界（中心在原点，按 size/2 作为墙位置）
//   const double x_min = -map_x_size / 2.0;
//   const double x_max =  map_x_size / 2.0;
//   const double y_min = -map_y_size / 2.0;
//   const double y_max =  map_y_size / 2.0;
//   const double z_min = 0.0;
//   const double z_max =  map_z_size;

//   // 立方体内部点采样步长
//   const double half_x = size_x / 2.0;
//   const double half_y = size_y / 2.0;
//   const double half_z = size_z / 2.0;

//   const double step_x = (pts_x > 1) ? (size_x / (pts_x - 1)) : size_x;
//   const double step_y = (pts_y > 1) ? (size_y / (pts_y - 1)) : size_y;
//   const double step_z = (pts_z > 1) ? (size_z / (pts_z - 1)) : size_z;

//   // 时间基准：2.0s 周期的分段加速度
//   const double cycle_T = 2.0;
//   ros::Time t0 = ros::Time::now();

//   auto accel_profile = [&](double t_mod) -> double {
//     if (t_mod < 1.0) return 3.0;       // 0 ~ 1.0 s: +3 m/s^2
//     if (t_mod < 1.2) return -30.0;     // 1.0 ~ 1.2 s: -30 m/s^2
//     return 3.0;                        // 1.2 ~ 2.0 s: +3 m/s^2
//   };

//   while (ros::ok()) {
//     // 当前周期内时间
//     double t = (ros::Time::now() - t0).toSec();
//     double t_mod = std::fmod(std::max(0.0, t), cycle_T);

//     // 分段加速度
//     double ax = accel_profile(t_mod);

//     // 半隐式欧拉积分：先速再位，方向可快速翻转
//     vx += ax * dt;
//     x  += vx * dt;

//     // 越界检查：重置到初始状态
//     if (x - half_x < x_min || x + half_x > x_max ||
//         y - half_y < y_min || y + half_y > y_max ||
//         z - half_z < z_min || z + half_z > z_max) {
//       x = start_x; y = start_y; z = start_z;
//       vx = init_speed;
//       t0 = ros::Time::now(); // 重置周期相位
//     }

//     // 构造立方体点云
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//     cloud->points.reserve(static_cast<size_t>(pts_x) * pts_y * pts_z);
//     for (int ix = 0; ix < pts_x; ++ix) {
//       for (int iy = 0; iy < pts_y; ++iy) {
//         for (int iz = 0; iz < pts_z; ++iz) {
//           pcl::PointXYZ p;
//           p.x = x + (-half_x + ix * step_x);
//           p.y = y + (-half_y + iy * step_y);
//           p.z = z + (-half_z + iz * step_z);
//           cloud->points.push_back(p);
//         }
//       }
//     }
//     cloud->width = cloud->points.size();
//     cloud->height = 1;
//     cloud->is_dense = true;

//     // 发布
//     const ros::Time now = ros::Time::now();
//     sensor_msgs::PointCloud2 msg;
//     pcl::toROSMsg(*cloud, msg);
//     msg.header.stamp = now;
//     msg.header.frame_id = frame_id;
//     obj_pub.publish(msg);

//     // 发布线速度（TwistStamped）
//     geometry_msgs::TwistStamped vel;
//     vel.header.stamp = now;
//     vel.header.frame_id = frame_id;
//     vel.twist.linear.x = vx;
//     vel.twist.linear.y = 0.0;
//     vel.twist.linear.z = 0.0;
//     vel.twist.angular.x = 0.0;
//     vel.twist.angular.y = 0.0;
//     vel.twist.angular.z = 0.0;
//     vel_pub.publish(vel);

//     ros::spinOnce();
//     rate.sleep();
//   }

//   return 0;
// }

// // 一个物体以正弦速度运动，周期为1秒，振幅为6.28米/秒
// int main(int argc, char** argv) {
//   ros::init(argc, argv, "ablation_map");
//   ros::NodeHandle nh("~");

//   std::string cloud_topic = "/map_generator/obj_cloud";
//   std::string frame_id = "world";
  
//   double rate_hz = 50.0;          // 发布频率
//   double speed = 0.0;             // 初始速度（m/s），沿 X 方向；保留旧参数名
//   double init_speed = speed;      // 可用 init_speed 参数覆盖
//   double size_x = 0.6, size_y = 0.8, size_z = 1.0; // 长方体三轴边长（米）
//   int pts_x = 5, pts_y = 5, pts_z = 13;            // 三轴采样点数
//   double start_x = 10.0, start_y = 0.0, start_z = 1.0; // 初始位置
//   double map_x_size = 35.0, map_y_size = 35.0, map_z_size = 5.0; // 地图边界，用于重置

//   std::string vel_topic = "/map_generator/obj_velocity";
//   nh.param("vel_topic", vel_topic, vel_topic);
//   ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(vel_topic, 1);
//   double vel_period = 1.0;      // s
//   double vel_amplitude = 6.28;  // m/s
//   double vel_phase = 0.0;       // rad，可选初相
//   double vel_bias = 0.0;        // m/s，速度直流偏置（如不需要保持0）
// //   nh.param("vel/period", vel_period, vel_period);
// //   nh.param("vel/amplitude", vel_amplitude, vel_amplitude);
// //   nh.param("vel/phase", vel_phase, vel_phase);
// //   nh.param("vel/bias", vel_bias, vel_bias);

//   nh.param("cloud_topic", cloud_topic, cloud_topic);
//   nh.param("frame_id", frame_id, frame_id);
//   nh.param("rate_hz", rate_hz, rate_hz);
//   nh.param("speed", speed, speed);                 // 兼容旧参数
//   nh.param("init_speed", init_speed, speed);       // 若提供 init_speed，则覆盖
//   nh.param("size_x", size_x, size_x);
//   nh.param("size_y", size_y, size_y);
//   nh.param("size_z", size_z, size_z);
//   nh.param("pts_x", pts_x, pts_x);
//   nh.param("pts_y", pts_y, pts_y);
//   nh.param("pts_z", pts_z, pts_z);
//   nh.param("start_x", start_x, start_x);
//   nh.param("start_y", start_y, start_y);
//   nh.param("start_z", start_z, start_z);
//   nh.param("map/x_size", map_x_size, map_x_size);
//   nh.param("map/y_size", map_y_size, map_y_size);
//   nh.param("map/z_size", map_z_size, map_z_size);

//   ros::Publisher obj_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 1);
//   ros::Rate rate(rate_hz);

//   const double dt = 1.0 / rate_hz;
//   double x = start_x;
//   double y = start_y;
//   double z = start_z;
//   double vx = init_speed; // X 轴速度

//   // 地图边界（中心在原点，按 size/2 作为墙位置）
//   const double x_min = -map_x_size / 2.0;
//   const double x_max =  map_x_size / 2.0;
//   const double y_min = -map_y_size / 2.0;
//   const double y_max =  map_y_size / 2.0;
//   const double z_min = 0.0;
//   const double z_max =  map_z_size;

//   // 立方体内部点采样步长
//   const double half_x = size_x / 2.0;
//   const double half_y = size_y / 2.0;
//   const double half_z = size_z / 2.0;

//   const double step_x = (pts_x > 1) ? (size_x / (pts_x - 1)) : size_x;
//   const double step_y = (pts_y > 1) ? (size_y / (pts_y - 1)) : size_y;
//   const double step_z = (pts_z > 1) ? (size_z / (pts_z - 1)) : size_z;

//   // 时间基准：2.0s 周期的分段加速度
//   ros::Time t0 = ros::Time::now();


//   while (ros::ok()) {
//     // 当前周期内时间
//     double t = (ros::Time::now() - t0).toSec();
//     double omega = (vel_period > 0.0) ? (2.0 * M_PI / vel_period) : 0.0;
//     // 正弦速度 vx(t) = bias + A * sin(ω t + φ)
//     vx = vel_bias + vel_amplitude * std::sin(omega * t + vel_phase);

//     // 半隐式欧拉积分：先速再位，方向可快速翻转
//     x  += vx * dt;

//     // 越界检查：重置到初始状态
//     if (x - half_x < x_min || x + half_x > x_max ||
//         y - half_y < y_min || y + half_y > y_max ||
//         z - half_z < z_min || z + half_z > z_max) {
//       x = start_x; y = start_y; z = start_z;
//       t0 = ros::Time::now(); // 重置相位
//     }

//     // 构造立方体点云
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//     cloud->points.reserve(static_cast<size_t>(pts_x) * pts_y * pts_z);
//     for (int ix = 0; ix < pts_x; ++ix) {
//       for (int iy = 0; iy < pts_y; ++iy) {
//         for (int iz = 0; iz < pts_z; ++iz) {
//           pcl::PointXYZ p;
//           p.x = x + (-half_x + ix * step_x);
//           p.y = y + (-half_y + iy * step_y);
//           p.z = z + (-half_z + iz * step_z);
//           cloud->points.push_back(p);
//         }
//       }
//     }
//     cloud->width = cloud->points.size();
//     cloud->height = 1;
//     cloud->is_dense = true;

//     // 发布
//     const ros::Time now = ros::Time::now();
//     sensor_msgs::PointCloud2 msg;
//     pcl::toROSMsg(*cloud, msg);
//     msg.header.stamp = now;
//     msg.header.frame_id = frame_id;
//     obj_pub.publish(msg);

//     // 发布线速度（TwistStamped）
//     geometry_msgs::TwistStamped vel;
//     vel.header.stamp = now;
//     vel.header.frame_id = frame_id;
//     vel.twist.linear.x = vx;
//     vel.twist.linear.y = 0.0;
//     vel.twist.linear.z = 0.0;
//     vel.twist.angular.x = 0.0;
//     vel.twist.angular.y = 0.0;
//     vel.twist.angular.z = 0.0;
//     vel_pub.publish(vel);

//     ros::spinOnce();
//     rate.sleep();
//   }

//   return 0;
// }