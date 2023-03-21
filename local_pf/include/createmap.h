#ifndef CREATEMAP_H
#define CREATEMAP_H
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/OccupancyGrid.h>
#include <time.h>
#include <boost/assign/list_of.hpp>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <future>
#include <tf/tf.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/MoveBaseResult.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <eigen3/Eigen/Dense>
//#include <Matrix3x3.h>
using namespace std;
using Eigen::MatrixXd;
using Eigen::Vector3d;
//typedef std::pair<int,int,int8_t> type_iii;
typedef std::pair<int,int> type_ii;
#define COLOR_NORMAL "\033[0m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"

namespace robot{

  class createmap
  {

  public:

    struct Quaternion
  {
      double w, x, y, z;
  };

    struct worldposition{
      double wx=0;
      double wy=0;
      double wtheta=0;
    };
    struct mapposition{
      int mx;
      int my;
      double mtheta;
    };
    struct lidardata{
      std::vector<type_ii> cells{(1,1)};
      std::vector<int> cellsvalue{0};
    };

    ros::NodeHandle rh;// robot handler!
    sensor_msgs::LaserScan laser_msg;
    nav_msgs::Odometry odom_msg;
    createmap::lidardata lid_data;
    std::vector<float> laser_ranges; //output of lidar_cb()
    geometry_msgs::Pose2D pose2d;

    timespec start_lcb;
    timespec start_ocb;
    timespec start_proclidar;
    timespec start_map;

    std::mutex lidar_mut;
    std::mutex map_mut;
    std::mutex odom_mut;


    std::condition_variable lidar_cv;
    std::condition_variable map_cv;
    std::condition_variable odom_cv;


    bool lidar_ready{false};
    bool map_ready{false};
    bool odom_ready{false};


    bool obstaclexist=false;
    float angle_min=0.0,angle_max=6.28318977,angle_increment=0.0175019,range_min=0.119,range_max=3.5;
    double origin_wx=200,origin_wy=200,origin_mx=-5,origin_my=-5;
    double mapres=0.05;
    int mapsize_x=(2*abs(origin_mx))/mapres,mapsize_y=mapsize_x;
    int laser_counter=0, odom_counter=0,counter2=0;
    createmap();
    virtual void init();
    Quaternion euler_quaternion(double yaw, double pitch, double roll); // yaw (Z), pitch (Y), roll (X)

    double time_elapsed(timespec& start, timespec& end);
    worldposition to_world(const int &mx,const int &my,const double &origin_wx,const double &origin_wy,const double res);

    mapposition to_map(const double &wx,const double &wy,const double &origin_mx,const double &origin_my,const int mapsize_x,const int mapsize_y,const double res);

    void Lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    void odom_cb (const nav_msgs::Odometry::ConstPtr& odom_msg);

    void update_map();

    void process_lidar();

    std::vector<int> findmapindex(const createmap::lidardata &lidarmsg);

    std::vector<type_ii> bresenham(const int& xr,const int& yr,const int& xl,const int& yl);
    geometry_msgs::Point lidarpoint_to_worldframe(int &ray_index,float &dist,float &angle_increment,geometry_msgs::Pose2D &robotpose2d);
~createmap();
  private:
  /* Threads*/





//    std::thread processlidar_th{&createmap::process_lidar,this}; // this keyboard is necessary to use inside the class as object
//    std::thread updatemap_th{&createmap::update_map,this}; // this keyboard is necessary to use inside the class as object

  /* end of Threads*/



//    std::thread updatemap_th{&createmap::update_map,this,std::ref(ld2)}; // this keyboard is necessary to use inside the class as object


    ros::Subscriber odom_sub =rh.subscribe("/odom", 40,&createmap::odom_cb,this); //new format of subscribing topic in constructer function of a class;
    ros::Subscriber lidar_sub=rh.subscribe("/scan", 10,&createmap::Lidar_cb,this); //new format of subscribing topic in constructer function of a class;
    ros::Publisher  map_pub  =rh.advertise<nav_msgs::OccupancyGrid>("map", 1);

  };

}
#endif // CREATEMAP_H
