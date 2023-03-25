#include "localization.h"
#include <memory.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

using namespace robot;

localization::localization()
{
 ROS_INFO("Start localization by initial values!");
init();

 //robot::createmap::init();
}
void localization:: init(){

  map_.info.resolution = 0.05;
  map_.info.origin.position.x = -10;
  map_.info.origin.position.y =-10;
  map_.info.width     = 384;
  map_.info.height    = 384;
  //first need to read lidar data becuase we need main feature of used lidar

}
localization::~localization(){
  EstimPos_th.join();
}
void localization::map_cb(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
  //cout<<"map recieved data!"<<endl;
  timespec end;
  clock_gettime(CLOCK_MONOTONIC_RAW, &end);
  //std:://cout<<"timelapsed in map callback function is:"<<(1.0/time_elapsed(this->start_mcb,end))<<" Hz"<<std::endl;


  try {
    std::lock_guard<std::mutex> map_lock(map_mut);

    {
      map_.header.seq = map_msg->header.seq;
      map_.header.frame_id = map_msg->header.frame_id;

      map_.info.resolution = map_msg->info.resolution;
      map_.info.width     = map_msg->info.width;
      map_.info.height    = map_msg->info.height;
      ///
      map_.info.origin.position.x = map_msg->info.origin.position.x;
      map_.info.origin.position.y =map_msg->info.origin.position.y;
      map_.info.origin.position.z = map_msg->info.origin.position.z;
      map_.info.origin.orientation.x = map_msg->info.origin.orientation.x;
      map_.info.origin.orientation.y = map_msg->info.origin.orientation.y;
      map_.info.origin.orientation.z = map_msg->info.origin.orientation.z;
      map_.info.origin.orientation.w = map_msg->info.origin.orientation.w;
      map_.data=map_msg->data;

//      if((1.0/time_elapsed(this->start_mcb,end)<1))// freq change under 10hz! the odom frequesncy shouldn't be higher than 20hz
//        throw std::exception();
      map_ready=true;

    }
    //cout<<"release the lock!!"<<endl;
    //cout<<map_.info.origin.position.x<<" , "<<map_.info.origin.position.y<<" , "<<map_.info.width<<" , "<<map_.info.height<<" , "<<map_.info.resolution<<endl;

       map_cv.notify_one();
  }
  catch (std::exception& e) {
    //cout << "Exception caught! map freq is decreased!" << endl;
  }
  //clock_gettime(CLOCK_MONOTONIC_RAW, &this->start_mcb);
}
// Goal is mapping a cell to a data with respect to the published map from map_server node!
int  localization::findmapindex(geometry_msgs::Pose &pnew){
    // mapping 2D map like (6,2) to 1D vector!
    // one cell to one index of a map array
      return ((pnew.position.y *map_.info.width)+pnew.position.x);  //  return (y*size+x);
  }
// each particle is a random cell, so it needs to be check whether is it inside the map or pointing to occupied cell!
bool localization::checkcell(geometry_msgs::Pose &pnew){
  // in this function the feasibility of robot position in this map will check
      if (pnew.position.x<0 ||  pnew.position.x>map_.info.width || pnew.position.y<0 || pnew.position.y>map_.info.height)
     {
    //cout<<"checkcell failed!"<<endl;
    return false;
  }

    int index=findmapindex(pnew);
// below condition means that particle shouldbe inside the known area of map!!
    if(map_.data.at(index)==100 || map_.data.at(index)==-1){
//      //cout<<"p5"<<endl;

      return false;
    }
    else{
//      //cout<<"p6"<<endl;
      return true;
    }

  }
// pose2d0 is real world intial position of robot, n is number of particles
geometry_msgs::PoseArray  localization::newparticles(const geometry_msgs::Pose2D &pose2d0, int n=10){
    createmap::mapposition rm_init;
    geometry_msgs::PoseArray partic;
    geometry_msgs::Pose pnew;
    rm_init=to_map(pose2d0.x,pose2d0.y,map_.info.origin.position.x,map_.info.origin.position.y,map_.info.width,map_.info.height,map_.info.resolution);
    normal_distribution<double> nidx(rm_init.mx,10);
    normal_distribution<double> nidy(rm_init.my,10);
    normal_distribution<double> nidtheta(rm_init.mtheta,1);

    unique_ptr<float> theta=make_unique<float>();

    //cout<<"rm_init.mx: "<<rm_init.mx;
    for (int i=0;i<n;++i) {
      do{
        pnew.position.x=int(nidx(mt))*1;
        pnew.position.y=int(nidy(mt))*1;
        *theta=nidtheta(mt);
        pnew.orientation.w=createmap::euler_quaternion(*theta,0,0).w;
        pnew.orientation.x=createmap::euler_quaternion(*theta,0,0).x;
        pnew.orientation.y=createmap::euler_quaternion(*theta,0,0).y;
        pnew.orientation.z=createmap::euler_quaternion(*theta,0,0).z;

      }while(!checkcell(pnew));
      partic.poses.push_back(pnew);
    }
    return partic;
};
template<typename T>
double localization:: calculate_distance(T x1, T y1, T x2, T y2) {
    double distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    return distance;
}


void localization::pub_particles(geometry_msgs::PoseArray &particlesongridmap){
  /* In this function the particles will publish on rviz screen.
   * Also by uncommenting the bolow lines its possible to see
   * the each particle expected scan on rviz by broadcasting the the particle frame,"expectescan", as child frame respect to "map"
   * as parent frame.
   */
  /*///Parametes
   * particleonworld: positions of all parties on real world frame
   * k: is particle number to be chose to visualize the expectec scan.
   * particleonworld2: position of  particle number k.
   * static_transformStamped: broadcast particle frame
   */
  int k=3;
  geometry_msgs::PoseArray particleonworld=particlesongridmap;
  geometry_msgs::PoseArray particleonworld2;
  createmap::worldposition wp;
  for(int i=0;i<(int)particlesongridmap.poses.size();++i){
    wp= createmap::to_world(particlesongridmap.poses.at(i).position.x,
                            particlesongridmap.poses.at(i).position.y,
                            origin_wx,origin_wy,map_.info.resolution);
    particleonworld.poses.at(i).position.x=wp.wx;
    particleonworld.poses.at(i).position.y=wp.wy;
  }
particleonworld2.poses.push_back(particleonworld.poses.at(k));
//  int* j=new int[0];
//  for (auto & i:particleonworld2.poses) {
//    (*j)++;
//    cout<<"particle number is :"<<*j<<" : "<<i.position.x<<" , "<<i.position.y<<" , qw: "<<i.orientation.w<<endl;
//  }

  particleonworld2.header.stamp=ros::Time::now();
  particleonworld2.header.frame_id="map";


  particles_pub.publish(particleonworld2);
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "map";
  static_transformStamped.child_frame_id = "expectedscan";
  static_transformStamped.transform.translation.x = particleonworld2.poses.at(0).position.x;
  static_transformStamped.transform.translation.y = particleonworld2.poses.at(0).position.y;
  static_transformStamped.transform.translation.z = 0;

  static_transformStamped.transform.rotation.x = particleonworld2.poses.at(0).orientation.x;
  static_transformStamped.transform.rotation.y = particleonworld2.poses.at(0).orientation.y;
  static_transformStamped.transform.rotation.z = particleonworld2.poses.at(0).orientation.z;
  static_transformStamped.transform.rotation.w = particleonworld2.poses.at(0).orientation.w;
  static_broadcaster.sendTransform(static_transformStamped);
//  delete[] j;

};

sensor_msgs::LaserScan localization::art_lidar(geometry_msgs::PoseArray& particle,sensor_msgs::LaserScan& lidar_msg,nav_msgs::OccupancyGrid& map_th){

  unique_ptr<sensor_msgs::LaserScan> lidmsg=make_unique<sensor_msgs::LaserScan>();
  unique_ptr<int> n=make_unique<int>(0);
  unique_ptr<vector<int>> count=make_unique<vector<int>>(0);

  unique_ptr<double> pitch=make_unique<double>(0),
                     roll=make_unique<double>(0);


  unique_ptr<tf::Quaternion> quat=make_unique<tf::Quaternion>();
  geometry_msgs::Pose2D particlepose2d;


  (*lidmsg).angle_min=      lidar_msg.angle_min;
  (*lidmsg).angle_max=      lidar_msg.angle_max;
  (*lidmsg).angle_increment=lidar_msg.angle_increment;
  (*lidmsg).range_min=      lidar_msg.range_min;
  (*lidmsg).range_max=      lidar_msg.range_max;
  //(*lidmsg).ranges=lidar_msg.ranges;
  ////cout<<"before While loop!"<<endl;
  int k=3;
  particlepose2d.x=particle.poses.at(k).position.x;
  particlepose2d.y=particle.poses.at(k).position.y;
  (*quat).setX(particle.poses.at(k).orientation.x);
  (*quat).setY(particle.poses.at(k).orientation.y);
  (*quat).setZ(particle.poses.at(k).orientation.z);
  (*quat).setW(particle.poses.at(k).orientation.w);
  tf::Matrix3x3(*quat).getRPY(*roll, *pitch, particlepose2d.theta);
//particlepose2d.theta=-particlepose2d.theta*0;
  cout<<"x: "<<particle.poses.at(k).orientation.x<<" , y: "<<particle.poses.at(k).orientation.y<<" , z: "<<particle.poses.at(k).orientation.z<<" , w: "<<particle.poses.at(k).orientation.w<<" , par yaw: "<<particlepose2d.theta*(180/M_PI)<<" , robot yaw: "<<pose2d.theta*(180/M_PI)<<endl;
  float dist=((*lidmsg).range_max)/map_.info.resolution;
  while(*n<(int)lidar_msg.ranges.size()){
//    unique_ptr<type_ii> cellsonray=make_unique<type_ii>();

    geometry_msgs::Point xyt=createmap::lidarpoint_to_worldframe(*n,dist,(*lidmsg).angle_increment,particlepose2d);

    //cout<<"x0:"<<particlepose2d.x<<" , y0:"<<particlepose2d.y<<" , xt:"<<xyt.x<<" , yt:"<<xyt.y<<endl;

    std::vector<type_ii> cellsonray=bresenham(particlepose2d.x,particlepose2d.y,xyt.x,xyt.y);
    int i=0;
    geometry_msgs::Pose* pnew=new geometry_msgs::Pose;
    geometry_msgs::Pose* old=new geometry_msgs::Pose;
    for (auto& c:cellsonray) {

      //cout<<"ray number is :"<<*n<<" , cellsonray.size() is : "<<cellsonray.size()<<" , cell number is : "<<i<<" , c.first is : "<<c.first<<" , c.second is: "<<c.second<<endl;
      i++;
//      if(*n>77){
//        (*lidmsg).ranges.push_back(1.0);
////        cout<<"inside cindition!"<<endl;
//        (*count).push_back( (*n)++);

//        break;
//      }
//      cout<<"outside cindition!"<<endl;

      (*pnew).position.x=c.first;
      (*pnew).position.y=c.second;
      int index=findmapindex(*pnew);
       //cout<<"index: " <<index<<", map_.data.at(index): "<<map_.data.at(index)<<endl;
      if(map_.data.at(index)==100 || map_.data.at(index)==-1){
        (*lidmsg).ranges.push_back(calculate_distance((int)particlepose2d.x,(int)particlepose2d.y,(int)(((*old).position.x+(*pnew).position.x)/2.0),(int)(((*old).position.y+(*pnew).position.y)/2.0))*map_.info.resolution);
        cout<<"n: "<<*n<<" , cell detected 1"<<endl;
        (*count).push_back( (*n)++);
        break;
      }
      if(i==(int)cellsonray.size()){
        cout<<"range max: "<<(*lidmsg).range_max<<endl;
        (*lidmsg).ranges.push_back((*lidmsg).range_max);
        cout<<"n: "<<*n<<" , cell detected 2"<<endl;
        (*count).push_back( (*n)++);
        break;
      }
      (*old).position.x=c.first;
      (*old).position.y=c.second;
    }
    delete pnew;
    delete old;
  }
  cout<<"count size:"<<(*count).size()<<endl;

  //cout<<"after While loop! n is :"<<*n<<endl;
  (*lidmsg).header.frame_id="expectedscan";

  return (*lidmsg);
}

void localization::estipose(){
  nav_msgs::OccupancyGrid map_th;
  geometry_msgs::PoseArray np; //number of particles
  std::vector<float> laser_ranges_; //output of lidar_cb()

  int laser_counter_=0;
  //float angle_min_=0.0,angle_max_=6.28318977,angle_increment_=0.0175019,range_min_=0.119,range_max_=3.5;

//  geometry_msgs::Pose2D pose2d1;
//  int n=50;
//  pose2d1.x=-2;
//  pose2d1.y=-0.5;
//  pose2d1.theta=0;
  //cout<<COLOR_GREEN<<"x: "<<pose2d.x<<" , "<<"y: "<<pose2d.y<<COLOR_NORMAL<<endl;
  {
    std::unique_lock<std::mutex> map_lock(map_mut);
    map_cv.wait(map_lock,[this](){return this->map_ready;});

    map_th.info.resolution = map_.info.resolution;
    map_th.info.origin.position.x = map_.info.origin.position.x;
    map_th.info.origin.position.y =map_.info.origin.position.y;
    map_th.info.width     = map_.info.width;
    map_th.info.height    = map_.info.height;
    map_th.data=map_.data;
    map_ready=false;
  }


  {
    std::unique_lock<std::mutex> lidar_lock(createmap::lidar_mut);
    createmap::lidar_cv.wait(lidar_lock,[this](){return createmap::lidar_ready;});
    lidar_msg.angle_min=      angle_min;
    lidar_msg.angle_max=      angle_max;
    lidar_msg.angle_increment=angle_increment;
    lidar_msg.range_min=      range_min;
    lidar_msg.range_max=      range_max;
    lidar_msg.ranges=  laser_ranges;
    laser_counter_=laser_counter;
    createmap::lidar_ready=false;
  }
  //cout<<"before creating particles"<<endl;
  np=newparticles(pose2d);
  //cout<<"before publishing particles"<<endl;
  // in order to publish the point cloud, the transform from grid map to world is necessary
  sensor_msgs::LaserScan exp_sc=art_lidar(np,lidar_msg,map_th);
  //cout<<"after publishing particles"<<endl;
  ros::Rate rate(1); // 10 Hz
  while (ros::ok())
  {
      pub_particles(np);
      expectedscanner_Pub.publish(exp_sc);
      //cout<<"particle 0: "<<np.poses.at(0).position.x<<" , "<<np.poses.at(0).position.y<<endl;

      rate.sleep();
  }

}
