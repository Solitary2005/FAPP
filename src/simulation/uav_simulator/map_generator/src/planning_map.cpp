// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <random>
// #include <cmath>
// #include <string>

// // 狭窄走廊环境生成器：两侧墙体 + 动态圆柱障碍物（双向）
// // - 走廊长度40m、宽3m，高度由 map/z_size 指定
// // - 动态障碍数量支持 10/30/50，模式同环境1（0恒速、1恒加速度、2正弦速度）
// // - 沿 y 轴速度为 0，沿 x 轴最大速度为 5.0m/s
// // - 半径随机 [0.2, 0.5]

// struct Obstacle {
//   double x, y, r;        // 位置与半径
//   double vx;             // 模式0/1使用
//   double ax;             // 模式1使用
//   double amp;            // 模式2：A
//   double omega;          // 模式2：ω
//   double phase;          // 模式2：相位（0或π），双向分流
// };

// int main(int argc, char **argv) {
//   ros::init(argc, argv, "planning_map");
//   ros::NodeHandle nh("~");

//   // 话题与频率
//   std::string frame_id = "world";
//   std::string global_topic = "/map_generator/global_cloud";
//   std::string wall_topic   = "/map_generator/wall_cloud";
//   std::string obj_topic    = "/map_generator/obj_cloud";
//   std::string static_topic = "/map_generator/static_obs_cloud"; // 一些模块订阅静态障碍
//   double rate_hz = 50.0;
//   nh.param("frame_id", frame_id, frame_id);
//   nh.param("global_topic", global_topic, global_topic);
//   nh.param("wall_topic", wall_topic, wall_topic);
//   nh.param("obj_topic", obj_topic, obj_topic);
//   nh.param("rate_hz", rate_hz, rate_hz);

//   // 走廊与地图参数
//   double corridor_len = 40.0; // m
//   double corridor_wid = 3.0;  // m
//   double map_z_size   = 3.0;  // m
//   nh.param("corridor/length", corridor_len, corridor_len);
//   nh.param("corridor/width", corridor_wid, corridor_wid);
//   nh.param("map/z_size", map_z_size, map_z_size);

//   // 采样分辨率
//   double wall_resolution = 0.2; // m
//   double obs_resolution  = 0.15; // m
//   nh.param("map/resolution", wall_resolution, wall_resolution);
//   nh.param("obstacle/resolution", obs_resolution, obs_resolution);

//   // 动态障碍参数
//   int moving_num = 10;
//   int moving_mode = 0; // 0:const v, 1:const acc, 2:sin vel
//   double r_min = 0.2, r_max = 0.5;
//   double max_speed_x = 5.0; // m/s（上限）
//   nh.param("moving_obs_num", moving_num, moving_num);
//   nh.param("moving_obs/mode", moving_mode, moving_mode);
//   nh.param("moving_obs/radius_min", r_min, r_min);
//   nh.param("moving_obs/radius_max", r_max, r_max);
//   nh.param("moving_obs/max_speed_x", max_speed_x, max_speed_x);

//   // 正弦速度参数范围（环境1一致）
//   double amp_l = 0.5, amp_h = 2.0;
//   double omega_l = 0.2, omega_h = 1.0;

//   // 加速度范围（环境1一致）
//   double acc_l = 0.0, acc_h = 1.5;

//   // 全局边界（走廊居中，沿x）
//   const double x_min = -corridor_len / 2.0;
//   const double x_max =  corridor_len / 2.0;
//   const double y_min = -corridor_wid / 2.0;
//   const double y_max =  corridor_wid / 2.0;
//   const double z_min = 0.0;
//   const double z_max =  map_z_size;

//   // 发布器
//   ros::Publisher global_pub = nh.advertise<sensor_msgs::PointCloud2>(global_topic, 1);
//   ros::Publisher wall_pub   = nh.advertise<sensor_msgs::PointCloud2>(wall_topic, 1);
//   ros::Publisher static_pub = nh.advertise<sensor_msgs::PointCloud2>(static_topic, 1);
//   ros::Publisher obj_pub    = nh.advertise<sensor_msgs::PointCloud2>(obj_topic, 1);

//   // 随机
//   std::random_device rd;
//   std::mt19937 rng(rd());
//   std::uniform_real_distribution<double> uni01(0.0, 1.0);
//   std::uniform_real_distribution<double> rand_r(r_min, r_max);
//   std::uniform_real_distribution<double> rand_x(x_min, x_max);
//   std::uniform_real_distribution<double> rand_y(y_min, y_max);
//   std::uniform_real_distribution<double> rand_speed(0.5, std::max(0.5, max_speed_x));
//   std::uniform_real_distribution<double> rand_acc(acc_l, acc_h);
//   std::uniform_real_distribution<double> rand_amp(amp_l, amp_h);
//   std::uniform_real_distribution<double> rand_omega(omega_l, omega_h);

//   // 生成墙体点云（两侧y=±width/2）
//   pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//   for (double x = x_min; x <= x_max + 1e-6; x += wall_resolution) {
//     for (double z = z_min; z <= z_max + 1e-6; z += wall_resolution) {
//       wall_cloud->points.emplace_back(pcl::PointXYZ{x, y_min, z});
//       wall_cloud->points.emplace_back(pcl::PointXYZ{x, y_max, z});
//     }
//   }
//   wall_cloud->width = wall_cloud->points.size();
//   wall_cloud->height = 1;
//   wall_cloud->is_dense = true;

//   // global = 墙体（也可拓展地面/顶面）
//   sensor_msgs::PointCloud2 wall_msg, global_msg, static_msg;
//   pcl::toROSMsg(*wall_cloud, wall_msg);
//   wall_msg.header.frame_id = frame_id;
//   global_msg = wall_msg; // 此处将墙体作为全局静态地图
//   static_msg = wall_msg; // 同步发布到静态障碍话题

//   // 动态障碍初始化
//   std::vector<Obstacle> objs;
//   objs.reserve(moving_num);
//   for (int i = 0; i < moving_num; ++i) {
//     Obstacle ob{};
//     ob.r = rand_r(rng);
//     // 初始位置：保持在走廊内侧，避开墙厚度（radius裕度）
//     std::uniform_real_distribution<double> rand_x_in(x_min + ob.r, x_max - ob.r);
//     std::uniform_real_distribution<double> rand_y_in(y_min + ob.r, y_max - ob.r);
//     ob.x = rand_x_in(rng);
//     ob.y = rand_y_in(rng);

//     // 双向分流：前半正向，后半反向
//     const bool forward = (i < moving_num / 2);

//     if (moving_mode == 0) {
//       // 恒速：vx ∈ [0.5, max_speed_x]，符号按分流
//       double vmag = rand_speed(rng);
//       ob.vx = forward ? +vmag : -vmag;
//       ob.ax = 0.0;
//       ob.amp = ob.omega = ob.phase = 0.0;
//     } else if (moving_mode == 1) {
//       // 恒加速度：ax ∈ [0, 1.5]，符号按分流；初速0，限速5.0
//       double amag = rand_acc(rng);
//       ob.ax = forward ? +amag : -amag;
//       ob.vx = 0.0;
//       ob.amp = ob.omega = ob.phase = 0.0;
//     } else {
//       // 正弦速度：vx = A*sin(ωt + φ)，A∈[0.5,2.0]，ω∈[0.2,1.0]，相位0/π
//       ob.amp = rand_amp(rng);
//       ob.omega = rand_omega(rng);
//       ob.phase = forward ? 0.0 : M_PI;
//       ob.vx = 0.0; ob.ax = 0.0;
//     }
//     objs.push_back(ob);
//   }

//   ros::Rate rate(rate_hz);
//   ros::Time t0 = ros::Time::now();
//   const double dt = 1.0 / std::max(1.0, rate_hz);

//   while (ros::ok()) {
//     const ros::Time now = ros::Time::now();
//     const double t = (now - t0).toSec();

//     // 发布静态地图（可每帧或仅启动时；这里每帧发布保证订阅者拿到）
//     wall_msg.header.stamp = now;
//     global_msg.header.stamp = now;
//     wall_pub.publish(wall_msg);
//     global_pub.publish(global_msg);
//     static_pub.publish(static_msg);

//     // 生成动态障碍点云
//     pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());

//     for (auto &ob : objs) {
//       // 速度更新与位置推进
//       if (moving_mode == 0) {
//         // 恒速
//         ob.x += ob.vx * dt;
//       } else if (moving_mode == 1) {
//         // 恒加速度 + 限速
//         ob.vx += ob.ax * dt;
//         if (ob.vx >  max_speed_x) ob.vx =  max_speed_x;
//         if (ob.vx < -max_speed_x) ob.vx = -max_speed_x;
//         ob.x += ob.vx * dt;
//       } else {
//         // 正弦速度
//         double vx = ob.amp * std::sin(ob.omega * t + ob.phase);
//         ob.x += vx * dt;
//       }

//       // 边界反弹（沿 x），保持在走廊内
//       const double xmin_r = x_min + ob.r;
//       const double xmax_r = x_max - ob.r;
//       if (ob.x < xmin_r) {
//         ob.x = xmin_r;
//         if (moving_mode == 0) ob.vx = -ob.vx;
//         else if (moving_mode == 1) { ob.vx = -ob.vx; ob.ax = -ob.ax; }
//         else { ob.phase += M_PI; }
//       } else if (ob.x > xmax_r) {
//         ob.x = xmax_r;
//         if (moving_mode == 0) ob.vx = -ob.vx;
//         else if (moving_mode == 1) { ob.vx = -ob.vx; ob.ax = -ob.ax; }
//         else { ob.phase += M_PI; }
//       }

//       // 构造圆柱（填充体素），z层高度可设为1.5m，底部抬高0.3m
//       double obs_height = std::min(1.5, z_max - 0.3);
//       double z_base = std::max(0.3, z_min);
//       for (double xx = ob.x - ob.r; xx <= ob.x + ob.r + 1e-6; xx += obs_resolution) {
//         for (double yy = ob.y - ob.r; yy <= ob.y + ob.r + 1e-6; yy += obs_resolution) {
//           double dx = xx - ob.x, dy = yy - ob.y;
//           if (dx*dx + dy*dy <= ob.r * ob.r) {
//             for (double zz = z_base; zz <= z_base + obs_height + 1e-6; zz += obs_resolution) {
//               obj_cloud->points.emplace_back(pcl::PointXYZ{(float)xx, (float)yy, (float)zz});
//             }
//           }
//         }
//       }
//     }

//     obj_cloud->width = obj_cloud->points.size();
//     obj_cloud->height = 1;
//     obj_cloud->is_dense = true;

//     sensor_msgs::PointCloud2 obj_msg;
//     pcl::toROSMsg(*obj_cloud, obj_msg);
//     obj_msg.header.stamp = now;
//     obj_msg.header.frame_id = frame_id;
//     obj_pub.publish(obj_msg);

//     ros::spinOnce();
//     rate.sleep();
//   }

//   return 0;
// }
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <cmath>
#include <string>

// 狭窄走廊环境生成器：两侧墙体 + 动态圆柱障碍物（双向）
// - 走廊长度40m、宽3m，高度由 map/z_size 指定
// - 动态障碍数量支持 10/30/50，模式同环境1（0恒速、1恒加速度、2正弦速度）
// - 沿 y 轴速度为 0，沿 x 轴最大速度为 5.0m/s
// - 半径随机 [0.2, 0.5]

struct Obstacle {
  double x, y, r;        // 位置与半径
  double vx;             // 模式0/1使用
  double ax;             // 模式1使用
  double amp;            // 模式2：A
  double omega;          // 模式2：ω
  double phase;          // 模式2：相位（0或π），双向分流
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "planning_map");
  ros::NodeHandle nh("~");

  // 话题与频率
  std::string frame_id = "world";
  std::string global_topic = "/map_generator/global_cloud";
  std::string wall_topic   = "/map_generator/wall_cloud";
  std::string obj_topic    = "/map_generator/obj_cloud";
  std::string static_topic = "/map_generator/static_obs_cloud"; // 一些模块订阅静态障碍
  double rate_hz = 50.0;
  nh.param("frame_id", frame_id, frame_id);
  nh.param("global_topic", global_topic, global_topic);
  nh.param("wall_topic", wall_topic, wall_topic);
  nh.param("obj_topic", obj_topic, obj_topic);
  nh.param("rate_hz", rate_hz, rate_hz);

  // 走廊与地图参数
  double corridor_len = 40.0; // m
  double corridor_wid = 3.0;  // m
  double map_z_size   = 3.0;  // m
  nh.param("corridor/length", corridor_len, corridor_len);
  nh.param("corridor/width", corridor_wid, corridor_wid);
  nh.param("map/z_size", map_z_size, map_z_size);

  // 采样分辨率
  double wall_resolution = 0.2; // m
  double obs_resolution  = 0.15; // m
  nh.param("map/resolution", wall_resolution, wall_resolution);
  nh.param("obstacle/resolution", obs_resolution, obs_resolution);

  // 动态障碍参数
  int moving_num = 10;
  int moving_mode = 0; // 0:const v, 1:const acc, 2:sin vel
  double r_min = 0.2, r_max = 0.5;
  double max_speed_x = 5.0; // m/s（上限）
  nh.param("moving_obs_num", moving_num, moving_num);
  nh.param("moving_obs/mode", moving_mode, moving_mode);
  nh.param("moving_obs/radius_min", r_min, r_min);
  nh.param("moving_obs/radius_max", r_max, r_max);
  nh.param("moving_obs/max_speed_x", max_speed_x, max_speed_x);
  bool convoy_mode = false;
  int convoy_num = 5;
  double convoy_speed_x = 0.6;
  double convoy_radius = 0.5;
  double convoy_spacing = 4.0;
  nh.param("scenario/convoy", convoy_mode, convoy_mode);
  nh.param("convoy/num", convoy_num, convoy_num);
  nh.param("convoy/speed_x", convoy_speed_x, convoy_speed_x);
  nh.param("convoy/radius", convoy_radius, convoy_radius);
  nh.param("convoy/spacing", convoy_spacing, convoy_spacing);

  // 正弦速度参数范围（环境1一致）
  double amp_l = 0.5, amp_h = 2.0;
  double omega_l = 0.2, omega_h = 1.0;

  // 加速度范围（环境1一致）
  double acc_l = 0.0, acc_h = 1.5;

  // 全局边界（走廊居中，沿x）
  const double x_min = -corridor_len / 2.0;
  const double x_max =  corridor_len / 2.0;
  const double y_min = -corridor_wid / 2.0;
  const double y_max =  corridor_wid / 2.0;
  const double z_min = 0.0;
  const double z_max =  map_z_size;

  // 发布器
  ros::Publisher global_pub = nh.advertise<sensor_msgs::PointCloud2>(global_topic, 1);
  ros::Publisher wall_pub   = nh.advertise<sensor_msgs::PointCloud2>(wall_topic, 1);
  ros::Publisher static_pub = nh.advertise<sensor_msgs::PointCloud2>(static_topic, 1);
  ros::Publisher obj_pub    = nh.advertise<sensor_msgs::PointCloud2>(obj_topic, 1);

  // 随机
  std::random_device rd;
  std::mt19937 rng(rd());
  std::uniform_real_distribution<double> uni01(0.0, 1.0);
  std::uniform_real_distribution<double> rand_r(r_min, r_max);
  std::uniform_real_distribution<double> rand_x(x_min, x_max);
  std::uniform_real_distribution<double> rand_y(y_min, y_max);
  std::uniform_real_distribution<double> rand_speed(0.5, std::max(0.5, max_speed_x));
  std::uniform_real_distribution<double> rand_acc(acc_l, acc_h);
  std::uniform_real_distribution<double> rand_amp(amp_l, amp_h);
  std::uniform_real_distribution<double> rand_omega(omega_l, omega_h);

  // 生成墙体点云（两侧y=±width/2）
  pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (double x = x_min; x <= x_max + 1e-6; x += wall_resolution) {
    for (double z = z_min; z <= z_max + 1e-6; z += wall_resolution) {
      wall_cloud->points.emplace_back(pcl::PointXYZ{x, y_min, z});
      wall_cloud->points.emplace_back(pcl::PointXYZ{x, y_max, z});
    }
  }
  wall_cloud->width = wall_cloud->points.size();
  wall_cloud->height = 1;
  wall_cloud->is_dense = true;

  // global = 墙体（也可拓展地面/顶面）
  sensor_msgs::PointCloud2 wall_msg, global_msg, static_msg;
  pcl::toROSMsg(*wall_cloud, wall_msg);
  wall_msg.header.frame_id = frame_id;
  global_msg = wall_msg; // 此处将墙体作为全局静态地图
  static_msg = wall_msg; // 同步发布到静态障碍话题

  // 动态障碍初始化
  std::vector<Obstacle> objs;
  objs.reserve(convoy_mode ? convoy_num : moving_num);

    if (convoy_mode) {
      // 并排横向排列（沿 y 方向分布），统一初始 x，沿 +x 同向前进
      moving_mode = 0; // 强制恒速模式
      const int N = convoy_num;
      const double r = convoy_radius;
      const double xmin_r = x_min + r;
      const double xmax_r = x_max - r;
      const double ymin_r = y_min + r;
      const double ymax_r = y_max - r;
      // 可用横向空间（扣除两侧墙的半径裕度）
      const double y_available = std::max(0.0, (ymax_r - ymin_r));
      const double y_step = (N > 1) ? (y_available / (N - 1)) : 0.0;
      const double y_start = -y_available / 2.0; // 居中分布

      for (int i = 0; i < N; ++i) {
        Obstacle ob{};
        ob.r = r;
        // 初始 x 统一（靠近左端），并排沿 y 分布
        ob.x = xmin_r + 0.5; // 留少许裕度，避免立刻循环
        double yi = y_start + i * y_step;
        // 夹紧到走廊有效范围内
        if (yi < ymin_r) yi = ymin_r;
        if (yi > ymax_r) yi = ymax_r;
        ob.y = yi;

        ob.vx = convoy_speed_x;  // 同向 0.6m/s
        ob.ax = 0.0;
        ob.amp = ob.omega = ob.phase = 0.0;
        objs.push_back(ob);
      }
    }
    else {
    for (int i = 0; i < moving_num; ++i) {
        Obstacle ob{};
        ob.r = rand_r(rng);
        // 初始位置：保持在走廊内侧，避开墙厚度（radius裕度）
        std::uniform_real_distribution<double> rand_x_in(x_min + ob.r, x_max - ob.r);
        std::uniform_real_distribution<double> rand_y_in(y_min + ob.r, y_max - ob.r);
        ob.x = rand_x_in(rng);
        ob.y = rand_y_in(rng);

        // 双向分流：前半正向，后半反向
        const bool forward = (i < moving_num / 2);

        if (moving_mode == 0) {
        // 恒速：vx ∈ [0.5, max_speed_x]，符号按分流
        double vmag = rand_speed(rng);
        ob.vx = forward ? +vmag : -vmag;
        ob.ax = 0.0;
        ob.amp = ob.omega = ob.phase = 0.0;
        } else if (moving_mode == 1) {
        // 恒加速度：ax ∈ [0, 1.5]，符号按分流；初速0，限速5.0
        double amag = rand_acc(rng);
        ob.ax = forward ? +amag : -amag;
        ob.vx = 0.0;
        ob.amp = ob.omega = ob.phase = 0.0;
        } else {
        // 正弦速度：vx = A*sin(ωt + φ)，A∈[0.5,2.0]，ω∈[0.2,1.0]，相位0/π
        ob.amp = rand_amp(rng);
        ob.omega = rand_omega(rng);
        ob.phase = forward ? 0.0 : M_PI;
        ob.vx = 0.0; ob.ax = 0.0;
        }
        objs.push_back(ob);
    }
    }

  ros::Rate rate(rate_hz);
  ros::Time t0 = ros::Time::now();
  const double dt = 1.0 / std::max(1.0, rate_hz);

  while (ros::ok()) {
    const ros::Time now = ros::Time::now();
    const double t = (now - t0).toSec();

    // 发布静态地图（可每帧或仅启动时；这里每帧发布保证订阅者拿到）
    wall_msg.header.stamp = now;
    global_msg.header.stamp = now;
    wall_pub.publish(wall_msg);
    global_pub.publish(global_msg);
    static_pub.publish(static_msg);

    // 生成动态障碍点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (auto &ob : objs) {
      if (convoy_mode) {
        // 统一恒速前进（同向），超界循环
        ob.x += ob.vx * dt;
        const double xmin_r = x_min + ob.r;
        const double xmax_r = x_max - ob.r;
        if (ob.x > xmax_r) {
          ob.x = xmin_r; // 循环到起点，保持同向
        }
      } else {
        // 速度更新与位置推进
        if (moving_mode == 0) {
          ob.x += ob.vx * dt;
        } else if (moving_mode == 1) {
          ob.vx += ob.ax * dt;
          if (ob.vx >  max_speed_x) ob.vx =  max_speed_x;
          if (ob.vx < -max_speed_x) ob.vx = -max_speed_x;
          ob.x += ob.vx * dt;
        } else {
          double vx = ob.amp * std::sin(ob.omega * t + ob.phase);
          ob.x += vx * dt;
        }

        // 边界反弹（沿 x），保持在走廊内
        const double xmin_r = x_min + ob.r;
        const double xmax_r = x_max - ob.r;
        if (ob.x < xmin_r) {
          ob.x = xmin_r;
          if (moving_mode == 0) ob.vx = -ob.vx;
          else if (moving_mode == 1) { ob.vx = -ob.vx; ob.ax = -ob.ax; }
          else { ob.phase += M_PI; }
        } else if (ob.x > xmax_r) {
          ob.x = xmax_r;
          if (moving_mode == 0) ob.vx = -ob.vx;
          else if (moving_mode == 1) { ob.vx = -ob.vx; ob.ax = -ob.ax; }
          else { ob.phase += M_PI; }
        }
      }

      // 构造圆柱（填充体素），z层高度可设为1.5m，底部抬高0.3m
      double obs_height = std::min(1.5, z_max - 0.3);
      double z_base = std::max(0.3, z_min);
      for (double xx = ob.x - ob.r; xx <= ob.x + ob.r + 1e-6; xx += obs_resolution) {
        for (double yy = ob.y - ob.r; yy <= ob.y + ob.r + 1e-6; yy += obs_resolution) {
          double dx = xx - ob.x, dy = yy - ob.y;
          if (dx*dx + dy*dy <= ob.r * ob.r) {
            for (double zz = z_base; zz <= z_base + obs_height + 1e-6; zz += obs_resolution) {
              obj_cloud->points.emplace_back(pcl::PointXYZ{(float)xx, (float)yy, (float)zz});
            }
          }
        }
      }
    }

    obj_cloud->width = obj_cloud->points.size();
    obj_cloud->height = 1;
    obj_cloud->is_dense = true;

    sensor_msgs::PointCloud2 obj_msg;
    pcl::toROSMsg(*obj_cloud, obj_msg);
    obj_msg.header.stamp = now;
    obj_msg.header.frame_id = frame_id;
    obj_pub.publish(obj_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
