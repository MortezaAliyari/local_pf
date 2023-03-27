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
geometry_msgs::PoseArray  localization::newparticles(const geometry_msgs::Pose2D &pose2d0, int n=100){
    createmap::mapposition rm_init;
    geometry_msgs::PoseArray partic;
    geometry_msgs::Pose pnew;
    rm_init=to_map(pose2d0.x,pose2d0.y,map_.info.origin.position.x,map_.info.origin.position.y,map_.info.width,map_.info.height,map_.info.resolution);
    normal_distribution<double> nidx(rm_init.mx,50);
    normal_distribution<double> nidy(rm_init.my,10);
    normal_distribution<double> nidtheta(rm_init.mtheta,6);

    unique_ptr<double> theta=make_unique<double>();

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
//        if(i==0){
//          pnew.position.x=rm_init.mx;
//          pnew.position.y=rm_init.my;
//          *theta=rm_init.mtheta;
//          pnew.orientation.w=createmap::euler_quaternion(*theta,0,0).w;
//          pnew.orientation.x=createmap::euler_quaternion(*theta,0,0).x;
//          pnew.orientation.y=createmap::euler_quaternion(*theta,0,0).y;
//          pnew.orientation.z=createmap::euler_quaternion(*theta,0,0).z;

//        }

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
  /*Parameters
   * particleonworld: positions of all parties on real world frame

   */
  geometry_msgs::PoseArray particleonworld=particlesongridmap;
  createmap::worldposition wp;
  for(int i=0;i<(int)particlesongridmap.poses.size();++i){
    wp= createmap::to_world(particlesongridmap.poses.at(i).position.x,
                            particlesongridmap.poses.at(i).position.y,
                            origin_wx,origin_wy,map_.info.resolution);
    particleonworld.poses.at(i).position.x=wp.wx;
    particleonworld.poses.at(i).position.y=wp.wy;
  }
//  int* j=new int[0];
//  for (auto & i:particleonworld2.poses) {
//    (*j)++;
//    cout<<"particle number is :"<<*j<<" : "<<i.position.x<<" , "<<i.position.y<<" , qw: "<<i.orientation.w<<endl;
//  }
// delete[] j;
  particleonworld.header.stamp=ros::Time::now();
  particleonworld.header.frame_id="map";
  particles_pub.publish(particleonworld);
};

void localization::pub_bestparticle(geometry_msgs::PoseArray &particlesongridmap,const int& particlenumber=0){
  /* In this function the particles will publish on rviz screen.
   * Also by uncommenting the bolow lines its possible to see
   * the each particle expected scan on rviz by broadcasting the the particle frame,"expectescan", as child frame respect to "map"
   * as parent frame.
   */
  /*Parametes
   * particleonworld: positions of all parties on real world frame
   * particlenumber: is particle number to be chose to visualize the expectec scan.
   * particleonworld2: position of  particle number .
   * static_transformStamped: broadcast particle frame
   */
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
  particleonworld2.poses.push_back(particleonworld.poses.at(particlenumber));
//  int* j=new int[0];
//  for (auto & i:particleonworld2.poses) {
//    (*j)++;
//    cout<<"particle number is :"<<*j<<" : "<<i.position.x<<" , "<<i.position.y<<" , qw: "<<i.orientation.w<<endl;
//  }
//  delete[] j;
  //j=NULL;
  particleonworld2.header.stamp=ros::Time::now();
  particleonworld2.header.frame_id="map";


  bestparticle_pub.publish(particleonworld2);
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
};
shared_ptr<vector<sensor_msgs::LaserScan>> localization::art_lidar(geometry_msgs::PoseArray& particle,sensor_msgs::LaserScan& lidar_msg,nav_msgs::OccupancyGrid& map_th){

  shared_ptr<vector<sensor_msgs::LaserScan>> expectedscans=make_shared<vector<sensor_msgs::LaserScan>>();
  unique_ptr<sensor_msgs::LaserScan> lidmsg=make_unique<sensor_msgs::LaserScan>();
  unique_ptr<int> raynumber=make_unique<int>(0),particlenumber=make_unique<int>(0);
  unique_ptr<vector<int>> count=make_unique<vector<int>>(0);
  unique_ptr<double> pitch=make_unique<double>(0),
                     roll=make_unique<double>(0);
  unique_ptr<float> dist=make_unique<float>(0);
  unique_ptr<tf::Quaternion> quat=make_unique<tf::Quaternion>();
  geometry_msgs::Pose2D particlepose2d;

  (*lidmsg).angle_min=      lidar_msg.angle_min;
  (*lidmsg).angle_max=      lidar_msg.angle_max;
  (*lidmsg).angle_increment=lidar_msg.angle_increment;
  (*lidmsg).range_min=      lidar_msg.range_min;
  (*lidmsg).range_max=      lidar_msg.range_max;
  (*lidmsg).header.frame_id="expectedscan";
  *dist=((*lidmsg).range_max)/map_.info.resolution;

  //(*lidmsg).ranges=lidar_msg.ranges;
  ////cout<<"before While loop!"<<endl;

  while(*particlenumber<(int)particle.poses.size()){

    particlepose2d.x=particle.poses.at(*particlenumber).position.x;
    particlepose2d.y=particle.poses.at(*particlenumber).position.y;
    (*quat).setX(particle.poses.at(*particlenumber).orientation.x);
    (*quat).setY(particle.poses.at(*particlenumber).orientation.y);
    (*quat).setZ(particle.poses.at(*particlenumber).orientation.z);
    (*quat).setW(particle.poses.at(*particlenumber).orientation.w);
    tf::Matrix3x3(*quat).getRPY(*roll, *pitch, particlepose2d.theta);
    while(*raynumber<360){
      geometry_msgs::Point xyt=createmap::lidarpoint_to_worldframe(*raynumber,*dist,(*lidmsg).angle_increment,particlepose2d);
      std::vector<type_ii> cellsonray=bresenham(particlepose2d.x,particlepose2d.y,xyt.x,xyt.y);
      unique_ptr<int> cellnumberofray=make_unique<int>(0);
      geometry_msgs::Pose* pnew=new geometry_msgs::Pose;
      geometry_msgs::Pose* old=new geometry_msgs::Pose;
      for (auto& c:cellsonray) {
        (*pnew).position.x=c.first;
        (*pnew).position.y=c.second;
        int index=findmapindex(*pnew);
        (*cellnumberofray)++;
        if(map_.data.at(index)==100 || map_.data.at(index)==-1){
          (*lidmsg).ranges.push_back(calculate_distance((int)particlepose2d.x,(int)particlepose2d.y,(int)(((*old).position.x+(*pnew).position.x)/2.0),(int)(((*old).position.y+(*pnew).position.y)/2.0))*map_.info.resolution);
//          (*lidmsg).ranges.push_back(calculate_distance((int)particlepose2d.x,(int)particlepose2d.y,(int)(*pnew).position.x,(int)(*pnew).position.y)*map_.info.resolution);

          (*count).push_back( (*raynumber)++);
          break;
        }
        if((*cellnumberofray)==(int)cellsonray.size()){
          (*lidmsg).ranges.push_back((*lidmsg).range_max);
          (*count).push_back( (*raynumber)++);
          break;
        }
        (*old).position.x=c.first;
        (*old).position.y=c.second;
      }
      delete pnew;
      pnew=NULL;
      delete old;
      old=NULL;

    }
    *raynumber=0;
    (*expectedscans).push_back(*lidmsg);
    (*lidmsg).ranges.clear();
    (*particlenumber)++;
  }

  return (expectedscans);
}

shared_ptr<vector<double>> localization::normalizedlikelihood(shared_ptr<vector<sensor_msgs::LaserScan>> expectedscans,sensor_msgs::LaserScan & measurement){
  shared_ptr<vector<sensor_msgs::LaserScan>> exp_scan=make_shared<vector<sensor_msgs::LaserScan>>();
  (*exp_scan)=*(expectedscans);
  shared_ptr<float> R=make_shared<float>(1);// measurement noise covariance
  shared_ptr<double> vhatsum=make_shared<double>(),qsum=make_shared<double>();
  shared_ptr<vector<double>> q=   make_shared<vector<double>>();
  shared_ptr<vector<double>> vhat=make_shared<vector<double>>();
  //create error vector
  for (int i=0;i<(int)exp_scan->size();++i) {
    std::transform((*exp_scan).at(i).ranges.begin(), (*exp_scan).at(i).ranges.end(),measurement.ranges.begin(),(*exp_scan).at(i).ranges.begin(), [](double a, double b) { return abs(a - b); });
    (*vhat).push_back(std::accumulate((*exp_scan).at(i).ranges.begin(), (*exp_scan).at(i).ranges.end(), 0.0));
    cout<<(*vhat).at(i)<<endl;
  }
cout<<"after loop"<<endl;
//@ normalized the error vector and calculated likelihood for each particle
//  *vhatsum=std::accumulate((*vhat).begin(), (*vhat).end(), 0.0);
//  for (int i=0;i<(int)exp_scan->size();++i) {
//    (*vhat).at(i)=(*vhat).at(i)/(*vhatsum);

//    cout<<"vhatsum: "<<(*vhatsum)<<", vhat: "<<(*vhat).at(i)<<" , "<<pow(2.71828,(-pow((*vhat).at(i), 2) / (2.0 * (*R))))<<endl;
//    (*q).push_back(100*((1 / (sqrt((*R))* sqrt(2.0*M_PI))) * pow(2.71828,-pow((*vhat).at(i), 2) / (2.0 * (*R)))));
//  }

//@ normalized likelihood for each particle
//  *qsum=std::accumulate((*q).begin(), (*q).end(), 0.0);
//  for (int i=0;i<(int)(*q).size();++i) {
//    (*q).at(i)=(*q).at(i)/(*qsum);
//  }

  return vhat;
}

void localization::estipose(){
  nav_msgs::OccupancyGrid map_th;
  geometry_msgs::PoseArray np; //number of particles
  std::vector<float> laser_ranges_; //output of lidar_cb()
  shared_ptr<vector<sensor_msgs::LaserScan>> expectedscans=make_shared<vector<sensor_msgs::LaserScan>>();

  int laser_counter_=0;

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
  np=newparticles(pose2d);
  cout<<"particle size is :"<<np.poses.size()<<endl;
  // in order to publish the point cloud, the transform from grid map to world is necessary
  expectedscans=art_lidar(np,lidar_msg,map_th);
  cout<<"Expectedscans size is :"<<(*expectedscans).size()<<endl;
// // cout<<(*expectedscans).at(1).ranges.at(10)<<" , "<<(*expectedscans).at(2).ranges.at(20)<<" , "<<(*expectedscans).at(3).ranges.at(30)<<" , "<<(*expectedscans).at(4).ranges.at(40)<<endl;
  shared_ptr<vector<double>> q=normalizedlikelihood(expectedscans,lidar_msg);
  int j=0;
  for (auto& i:*q) {
    cout<<"q"<<j<< ": "<<i<<endl;
    j++;
  }
  cout<<"smallest error is: "<<*min_element((*q).begin(),(*q).end())<<", index is : "
      <<min_element((*q).begin(),(*q).end())-(*q).begin()<<endl;

  ros::Rate rate(1); // 10 Hz
  while(ros::ok()) {
      pub_particles(np);
      pub_bestparticle(np,min_element((*q).begin(),(*q).end())-(*q).begin());
      expectedscanner_Pub.publish((*expectedscans).at(min_element((*q).begin(),(*q).end())-(*q).begin()));
      rate.sleep();
  }
//  while(1);

}
