#include <ros/ros.h>
#include <iostream>
#include "localization.h"

using namespace std;
using namespace robot;
int main(int argc, char** argv){

  ros::init(argc, argv, "mainlocal");
  localization lc;
//  geometry_msgs::Pose2D pose2d1; int n=10;
//  vector<createmap::mapposition> v;
//  pose2d1.x=-2;
//  pose2d1.y=-0.5;
//  pose2d1.theta=0;
//  cout<<"befor newparticle function"<<endl;
//  v=lc.newparticles(pose2d1,n);
//  for (auto & i:v) {
//    cout<<i.mx<<" , "<<i.my<<" , "<<i.mtheta<<endl;
//  }



  ros::spin();

  return(0);
}
