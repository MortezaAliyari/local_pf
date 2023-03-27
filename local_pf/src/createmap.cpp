#include "createmap.h"
#include<stdexcept>
#include<boost/assign/list_inserter.hpp>
#include<cmath>
using namespace robot;
// cretemap class;

  createmap::Quaternion createmap::euler_quaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
  {
      // Abbreviations for the various angular functions
      double cy = std::cos(yaw * 0.5);
      double sy = std::sin(yaw * 0.5);
      double cp = std::cos(pitch * 0.5);
      double sp = std::sin(pitch * 0.5);
      double cr = std::cos(roll * 0.5);
      double sr = std::sin(roll * 0.5);

      createmap::Quaternion q;
      q.w = cr * cp * cy + sr * sp * sy;
      q.x = sr * cp * cy - cr * sp * sy;
      q.y = cr * sp * cy + sr * cp * sy;
      q.z = cr * cp * sy - sr * sp * cy;

      return q;
  };

  createmap::createmap()
  {
      //ROS_INFO("Start initialization!");
     // init();
  };
  void createmap::init()
  {
    //cout<<"createmap::init()"<<endl;
   // max_range=5; //lidar max range
  //  this_thread::sleep_for(std::chrono::seconds(1));

  //  updatemap_th.join();
  };
  double createmap::time_elapsed(timespec& start, timespec& end)
  {
      return ((1e9 * end.tv_sec + end.tv_nsec) -
              (1e9 * start.tv_sec + start.tv_nsec)) / 1.0e9;
  }
  createmap::worldposition createmap::to_world(const int &mx,const int &my,const double &origin_wx,
                                               const double &origin_wy,const double res){
    // origin_wx : the world frame in map world 200 ->y axis
    // origin_wy : the world frame in map world 200 -> xaxis
    // mx: relate to row number-> y axis
    // my: relate to row number-> x axis
    try {
      worldposition wpos;
      MatrixXd cz(3,3);
      Vector3d v;
      cz(0,0)=cos(M_PI/2);
      cz(0,1)=sin(M_PI/2);
      cz(0,2)=0;
      cz(1,0)=-sin(M_PI/2);
      cz(1,1)=cos(M_PI/2);
      cz(1,2)=0;
      cz(2,0)=0;
      cz(2,1)=0;
      cz(2,2)=1;
//      if(mx>mapsize_x || my>mapsize_y || mx<0 || my<0)
//        throw std::invalid_argument("robot's map location is not correct!! ");
      v[1]=(mx-origin_wx+0.5)*res;
      v[0]=-(my-origin_wy+0.5)*res;
      v[2]=0;
      wpos.wx=(cz * v)[0];
      wpos.wy=(cz * v)[1];
      wpos.wtheta=(cz * v)[2];
      return wpos;

    } catch (std::invalid_argument& e) {
      worldposition wpos;

      ROS_INFO("[%s]",e.what());
      return wpos;
    }

  };
  createmap::mapposition createmap::to_map(const double &wx,const double &wy,const double &origin_mx,
                                           const double &origin_my,const int mapsize_x,const int mapsize_y,const double res){
    // origin_mx : the map frame in real world -10
    // origin_my : the map frame in real world -10
    // wx: position on x axis
    // wy: position on y axis
    try {
      mapposition mpos;

      if (wx<origin_mx || wy<origin_my)
        throw std::invalid_argument("robot's real location is not correct!! ");

      mpos.mx=abs(wx-origin_mx)/res;
      mpos.my=abs(wy-origin_my)/res;
      if (mpos.mx>=mapsize_x || mpos.my>=mapsize_y)
        throw std::invalid_argument("robot's location on map is not correct!! ");
      return mpos;
    } catch (std::invalid_argument& e)
    {
      mapposition mpos;
      ROS_INFO("[%s]",e.what());
      return mpos;
    }


  }
  void createmap::odom_cb(const nav_msgs::Odometry::ConstPtr& odom_msg){
    timespec end;
    clock_gettime(CLOCK_MONOTONIC_RAW, &end);
//    std::cout<<"timelapsed in odom callback function is:"<<(1.0/this->time_elapsed(this->start_ocb,end))<<" Hz"<<std::endl;
    tf::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    try {
      std::lock_guard<std::mutex> odom_lock(odom_mut);

      {

        pose2d.x = odom_msg->pose.pose.position.x;
        pose2d.y = odom_msg->pose.pose.position.y;
        pose2d.theta = yaw;
        odom_counter=odom_msg->header.seq;

        if((1.0/this->time_elapsed(this->start_ocb,end)<18))// freq change under 10hz! the odom frequesncy shouldn't be higher than 20hz
          throw std::exception();
        odom_ready=true;

      }

         odom_cv.notify_one();
    }
    catch (std::exception& e) {
      cout << "Exception caught! Odom freq is decreased!" << endl;
    }
    clock_gettime(CLOCK_MONOTONIC_RAW, &this->start_ocb);

  }
  void createmap::Lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
    // Read and process laser scan values

        timespec end;
        clock_gettime(CLOCK_MONOTONIC_RAW, &end);
  //      std::cout<<"timelapsed in lidar callback function is:"<<(1.0/this->time_elapsed(this->start_lcb,end))<<" Hz"<<std::endl;

    try{

        {
        //cout<<"lock lidar mut inside callback!"<<endl;
        std::lock_guard<std::mutex> lidar_lock(lidar_mut);
        laser_msg = *scan_msg;
        angle_min=      laser_msg.angle_min;
        angle_max=      laser_msg.angle_max;
        angle_increment=laser_msg.angle_increment;
        range_min=      laser_msg.range_min;
        range_max=      laser_msg.range_max;
        laser_ranges =  laser_msg.ranges;
        for (int i=0;i<laser_ranges.size();++i) {
          if(laser_ranges.at(i)==INFINITY)
            laser_ranges.at(i)=range_max;
        }
        laser_counter=laser_msg.header.seq;
       if(  ((1.0/(this->time_elapsed(this->start_lcb,end))))<=4.5)// freq change under 4.5hz!
         throw std::exception();
        lidar_ready=true;

      }
      lidar_cv.notify_one();

      }
    catch (std::exception& e) {
      cout << "Exception caught! Lidar freq is decreased!" << endl;
    }
        clock_gettime(CLOCK_MONOTONIC_RAW, &this->start_lcb);

  }
  std::vector<type_ii> createmap::bresenham(const int& xr,const int& yr,const int& xl,const int& yl){

    float dx=0.0, dy=0.0; // tip: this variables should be float otherwise division would not be float!
    int x_new=0, y_new=0;
    float m=1;
    typedef std::pair<int,int> type_ii;

    std::vector<type_ii> lidar_line;
    dx=xl-xr;
    dy=yl-yr;
    x_new=xr;
    y_new=yr;
    if(dx>0){
      m=dy/dx;//

      while(x_new<xl){
        x_new++;
        y_new=int(m*(x_new-xr)+yr);

        boost::assign::push_back(lidar_line)(x_new,y_new);

      }
    }
    else if (dx<0) {
      m=dy/dx;

      while(x_new>xl){
        x_new--;
        y_new=int(m*(x_new-xr)+yr);
        boost::assign::push_back(lidar_line)(x_new,y_new);

      }
    }
    else {
      if(yr>yl){
        while(y_new>yl){
          y_new--;
          boost::assign::push_back(lidar_line)(x_new,y_new);
        }
      }
      else if (yr<yl) {
        while(y_new<yl){
          y_new++;
          boost::assign::push_back(lidar_line)(x_new,y_new);
        }
      }
      else {
        return lidar_line;
      }
    }

    return lidar_line;
    }

  std::vector<int>  createmap::findmapindex(const createmap::lidardata &lidarmsg){
    // mapping 2D map like (6,2) to 1D vector!
    std::vector<int> onedim_data;
    auto size=mapsize_x;
    for (auto& i:lidarmsg.cells)
      onedim_data.push_back(i.second*size+i.first);


  //  return (y*size+x);
    return onedim_data;
  }

  geometry_msgs::Point createmap::lidarpoint_to_worldframe(int &ray_index,float &dist,float &angle_increment,geometry_msgs::Pose2D &robotpose2d){
    geometry_msgs::Point point_sc2w,point_scan;
    geometry_msgs::TransformStamped transformStamped;
    MatrixXd cz(3,3);
    Vector3d v;

    v[0]=dist*cos(ray_index*angle_increment);
    v[1]=dist*sin(ray_index*angle_increment);
    v[2]=0;
    //transformed_scan.point=point_scan;
    //geometry_msgs::TransformStamped transformStamped;
    cz(0,0)=cos(-robotpose2d.theta);
    cz(0,1)=sin(-robotpose2d.theta);
    cz(0,2)=0;
    cz(1,0)=-sin(-robotpose2d.theta);
    cz(1,1)=cos(-robotpose2d.theta);
    cz(1,2)=0;
    cz(2,0)=0;
    cz(2,1)=0;
    cz(2,2)=1;
  //  tf2_ros::Buffer tfBuffer;
  //  tf2_ros::TransformListener tfListener(tfBuffer);
  //  try{
  //    transformStamped = tfBuffer.lookupTransform("base_link","base_scan",ros::Time(0));

  //  }
  //  catch (tf2::TransformException &ex) {
  //    ROS_WARN("%s",ex.what());
  //    ros::Duration(1.0).sleep();

  //  }
  //  point_sc2w.x= transformStamped.transform.translation.x;
  //  point_sc2w.y= transformStamped.transform.translation.y;
  //  point_sc2w.z= transformStamped.transform.translation.z;

    point_sc2w.x= (cz * v)[0]+robotpose2d.x-0.03;
    point_sc2w.y= (cz * v)[1]+robotpose2d.y;
    point_sc2w.z= (cz * v)[2]+0;
    return point_sc2w;
  }

  void createmap::process_lidar(){
    std::vector<float> laser_ranges_; //output of lidar_cb()
    int laser_counter_=0,odom_counter_=0;
    float angle_min_=0.0,angle_max_=6.28318977,angle_increment_=0.0175019,range_min_=0.119,range_max_=3.5;
    geometry_msgs::Pose2D pose2d_;

    while(1){
      int ind=0;
      createmap::lidardata lid_data_;
  //first need to read lidar data becuase it has lower rate ->lidar freq is almost 5hz
      {
        std::unique_lock<std::mutex> prolidar_lock(lidar_mut);// lidar freq is almost 5hz
        lidar_cv.wait(prolidar_lock,[this](){return this->lidar_ready;});
        angle_min_=      angle_min;
        angle_max_=      angle_max;
        angle_increment_=angle_increment;
        range_min_=      range_min;
        range_max_=      range_max;
        laser_ranges_=  laser_ranges;
        laser_counter_=laser_counter;
        lidar_ready=false;// this is neccessary to change inside the scope to prevent the race condition!
      }
  // second step is reading Odom info becuase it has higher rate and depens to dynamic of AMR
      {
        std::unique_lock<std::mutex> odom_lock(odom_mut);
        odom_cv.wait(odom_lock,[this](){return this->odom_ready;});// odom freq is almost 20hz!
        pose2d_=pose2d;
        odom_counter_=odom_counter;
        odom_ready=false;// this is neccessary to change inside the scope to prevent the race condition!
      }

      for (auto& dist:laser_ranges_) {
          if(dist<range_max){
            lid_data_.cellsvalue.push_back(100);
          }
          else if ( dist>range_max) {
             dist=range_max;
             lid_data_.cellsvalue.push_back(0);
          }
          else {
            lid_data_.cellsvalue.push_back(0);

          }

          lid_data_.cells.push_back({to_map(lidarpoint_to_worldframe(ind,dist,angle_increment_,pose2d_).x,
                                    lidarpoint_to_worldframe(ind,dist,angle_increment_,pose2d_).y,
                                    origin_mx,
                                    origin_my,
                                    mapsize_x,
                                    mapsize_y,
                                    mapres).mx,
                                    to_map(lidarpoint_to_worldframe(ind,dist,angle_increment_,pose2d_).x,
                                    lidarpoint_to_worldframe(ind,dist,angle_increment_,pose2d_).y,
                                    origin_mx,
                                    origin_my,
                                    mapsize_x,
                                    mapsize_y,
                                    mapres).my});
          ind++;
        }
      lid_data_.cells.at(0)={to_map(pose2d_.x,pose2d_.y,origin_mx,origin_my,
                                mapsize_x,
                                mapsize_y,
                                mapres).mx,
                                to_map(pose2d_.x,pose2d_.y,origin_mx,origin_my,
                                mapsize_x,
                                mapsize_y,
                                mapres).my};
      {
        std::unique_lock<std::mutex> map_lock(map_mut);
        lid_data=lid_data_;
        map_ready=true;
      }
      map_cv.notify_one();

    }

  }

  createmap::~createmap(){

    //processlidar_th.join();
   // updatemap_th.join();

  }
  void createmap::update_map(){
//  timespec start,end;
  nav_msgs::OccupancyGrid map_;
  geometry_msgs::Pose2D pose2d_;
  map_.header.seq = 0;
  map_.header.frame_id = "map";

  map_.info.resolution = mapres;
  map_.info.width     = mapsize_x;
  map_.info.height    = mapsize_y;
  ///
  map_.info.origin.position.x = origin_mx;
  map_.info.origin.position.y =origin_my;
  map_.info.origin.position.z = 0.0;
  map_.info.origin.orientation.x = 0.0;
  map_.info.origin.orientation.y = 0.0;
  map_.info.origin.orientation.z = 0.0;
  map_.info.origin.orientation.w = 0.0;
  map_.data.resize(map_.info.width * map_.info.height);
  std::vector<int8_t> a(map_.data.size(),-1);
  int p=0;

  while(1){
//    clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    int j=0;

    {
      std::unique_lock<std::mutex> odom_lock(odom_mut);
      odom_cv.wait(odom_lock,[this](){return this->odom_ready;});// odom freq is almost 20hz!
      pose2d_=pose2d;
      odom_ready=false;// this is neccessary to change inside the scope to prevent the race condition!
    }

    createmap::lidardata data;

    {
      std::unique_lock<mutex> map_lock(map_mut);
      map_cv.wait(map_lock,[this](){return this->map_ready;});
      data=std::move(lid_data);
      map_ready=false;
    }
    for (auto & i:data.cells) {
    std::vector<type_ii> br= bresenham(data.cells.at(0).first,data.cells.at(0).second,i.first,i.second);
    data.cells.insert(data.cells.end(),br.begin(),br.end());

    }



  // this thread will wait for list of index from process_lidar thread to update map!
  std::vector<int> indexes=findmapindex(data);
  for(auto& ind:indexes){
    if(a.at(ind)==-1){
      if(j<361)
      a.at(ind) =data.cellsvalue.at(j);
      else
        a.at(ind)=0;
    }
    j++;

  }
  map_.data=a;
  map_.header.stamp = ros::Time::now();
  map_.info.map_load_time=ros::Time::now();
  map_pub.publish(map_);

//  this_thread::sleep_for(chrono::milliseconds(200));

    //    clock_gettime(CLOCK_MONOTONIC_RAW, &end);
    //    std::cout<<"timelapsed in map thread function is:"<<1.0/(this->time_elapsed(start,end))<<" Hz"<<std::endl;
 }
}
