#include "localization.h"
#include<memory.h>
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
  cout<<"map recieved data!"<<endl;
  timespec end;
  clock_gettime(CLOCK_MONOTONIC_RAW, &end);
  std::cout<<"timelapsed in map callback function is:"<<(1.0/time_elapsed(this->start_mcb,end))<<" Hz"<<std::endl;


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
    cout<<"release the lock!!"<<endl;
    cout<<map_.info.origin.position.x<<" , "<<map_.info.origin.position.y<<" , "<<map_.info.width<<" , "<<map_.info.height<<" , "<<map_.info.resolution<<endl;

       map_cv.notify_one();
  }
  catch (std::exception& e) {
    cout << "Exception caught! map freq is decreased!" << endl;
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
    cout<<"checkcell failed!"<<endl;
    return false;
  }

    int index=findmapindex(pnew);

    if(map_.data.at(index)==100){
//      cout<<"p5"<<endl;

      return false;
    }
    else{
//      cout<<"p6"<<endl;
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
    normal_distribution<double> nidtheta(rm_init.mtheta,3.14);

    createmap::worldposition nw;

    cout<<"rm_init.mx: "<<rm_init.mx;
    for (int i=0;i<n;++i) {
      do{
        pnew.position.x=int(nidx(mt))*1;
        pnew.position.y=int(nidy(mt))*1;
        float theta=rm_init.mtheta+nidtheta(mt);
        pnew.orientation.w=createmap::euler_quaternion(theta,0,0).w;
        pnew.orientation.x=createmap::euler_quaternion(theta,0,0).x;
        pnew.orientation.y=createmap::euler_quaternion(theta,0,0).y;
        pnew.orientation.z=createmap::euler_quaternion(theta,0,0).z;

      }while(!checkcell(pnew));
      partic.poses.push_back(pnew);
    }
    return partic;
};



void localization::pub_particles(geometry_msgs::PoseArray &particlesongridmap){

  geometry_msgs::PoseArray particleonworld=particlesongridmap;
  createmap::worldposition wp;
  for(int i=0;i<(int)particlesongridmap.poses.size();++i){
    wp= createmap::to_world(particlesongridmap.poses.at(i).position.x,
                            particlesongridmap.poses.at(i).position.y,
                            origin_wx,origin_wy,map_.info.resolution);
    particleonworld.poses.at(i).position.x=wp.wx;
    particleonworld.poses.at(i).position.y=wp.wy;
  }

  int* j=new int(1);
  for (auto & i:particleonworld.poses) {
    (*j)++;
    cout<<*j<<" : "<<i.position.x<<" , "<<i.position.y<<endl;
  }

  particleonworld.header.stamp=ros::Time::now();
  particleonworld.header.frame_id="map";
  particles_pub.publish(particleonworld);
  delete[] j;

};

sensor_msgs::LaserScan localization::art_lidar(geometry_msgs::PoseArray& particle,sensor_msgs::LaserScan& lidar_msg,nav_msgs::OccupancyGrid& map_th){
  unique_ptr<sensor_msgs::LaserScan> lidmsg=make_unique<sensor_msgs::LaserScan>();
  unique_ptr<uint16_t> n=make_unique<uint16_t>(1);
 //sensor_msgs::LaserScan exp_scan;

  (*lidmsg).angle_min=      lidar_msg.angle_min;
  (*lidmsg).angle_max=      lidar_msg.angle_max;
  (*lidmsg).angle_increment=lidar_msg.angle_increment;
  (*lidmsg).range_min=      lidar_msg.range_min;
  (*lidmsg).range_max=      lidar_msg.range_max;
  (*lidmsg).ranges=lidar_msg.ranges;
  cout<<"before While loop!"<<endl;

  while(*n<=lidar_msg.ranges.size()){
    (*n)++;



  }
  cout<<"after While loop!"<<endl;
  (*lidmsg).header.frame_id="base_scan";
  return (*lidmsg);
}

void localization::estipose(){
  nav_msgs::OccupancyGrid map_th;
  geometry_msgs::PoseArray np; //number of particles
  std::vector<float> laser_ranges_; //output of lidar_cb()

  int laser_counter_=0,map_counter_=0,j=0;
  float angle_min_=0.0,angle_max_=6.28318977,angle_increment_=0.0175019,range_min_=0.119,range_max_=3.5;

//  geometry_msgs::Pose2D pose2d1;
//  int n=50;
//  pose2d1.x=-2;
//  pose2d1.y=-0.5;
//  pose2d1.theta=0;
  cout<<COLOR_GREEN<<"x: "<<pose2d.x<<" , "<<"y: "<<pose2d.y<<COLOR_NORMAL<<endl;
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
  cout<<"before creating particles"<<endl;
    np=newparticles(pose2d);
    cout<<"before publishing particles"<<endl;
    // in order to publish the point cloud, the transform from grid map to world is necessary
    pub_particles(np);
    ros::Rate rate(10); // 10 Hz
    while (ros::ok())
    {
        // Publish LIDAR data
        sensor_msgs::LaserScan lidar_data;
        // Fill in the LIDAR data here...
        expectedscanner_Pub.publish(art_lidar(np,lidar_msg,map_th));

//        ros::spinOnce();
        rate.sleep();
    }

}
