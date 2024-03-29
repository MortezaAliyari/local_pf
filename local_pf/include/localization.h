#ifndef LOCALIZATION_H
#define LOCALIZATION_H
#include "ros/ros.h"
#include "ros/console.h"
#include "createmap.h"
#include <random>
#include "geometry_msgs/PoseArray.h"

namespace robot {
class localization: public robot::createmap
{
public:

  localization();
  ~localization();
  void init();
  void map_cb(const nav_msgs::OccupancyGrid::ConstPtr & map_msg);
  geometry_msgs::PoseArray  newparticles(const geometry_msgs::Pose2D &pose2d0, int n);
  void estipose();
  int  findmapindex(geometry_msgs::Pose &pnew);

  bool checkcell(geometry_msgs::Pose &pnew);
  // publish particles and check the convergence of the distribution visually by RVIZ.

  void pub_particles(geometry_msgs::PoseArray &particlesongridmap);
  // Publish expected lidar scan or artificial lidar scan per particle to RVIZ to chekc the data visually
  sensor_msgs::LaserScan art_lidar(geometry_msgs::PoseArray& particle,sensor_msgs::LaserScan& lidar_msg,nav_msgs::OccupancyGrid& map_th);
protected: // in some cases need to inhertence the variables of base class so they should be protected.
  std::mt19937 mt;
  nav_msgs::OccupancyGrid map_;


  std::mutex map_mut;

  std::condition_variable map_cv;

  bool map_ready{false};
  timespec start_mcb;
  sensor_msgs::LaserScan lidar_msg;

private:// in some cases, inhertence of below functions and threads from base class are not allowded,so they should be private.
  std::thread EstimPos_th{&localization::estipose,this}; // "this" keyboard is necessary to use inside the class as object
  // publish particles and check the convergence of the distribution visually by RVIZ.
  ros::Publisher  particles_pub  =rh.advertise<geometry_msgs::PoseArray>("/particlecloud", 1);
  ros::Publisher expectedscanner_Pub=rh.advertise<sensor_msgs::LaserScan>("expected_scanner",1);


  ros::Subscriber odom_sub =createmap::rh.subscribe("/map", 1,&localization::map_cb,this); //new format of subscribing topic in constructer function of a class;
};

}

#endif // LOCALIZATION_H
