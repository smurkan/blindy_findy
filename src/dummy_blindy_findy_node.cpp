
#include <ros/ros.h>
#include "blindy_findy/distances.h"
#include <unistd.h>


int main(int argc, char** argv)
{ 
  float distVal[3];
  distVal[0] = 1.73;
  distVal[1] = 2.54;
  distVal[2] = 5.32;

  ros::init(argc, argv, "blindy_findy");
  ros::NodeHandle nh;
  
  ros::Publisher pub = nh.advertise<blindy_findy::distances>("distances", 1);
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    blindy_findy::distances dmsg;
    dmsg.distL = distVal[0];
    dmsg.distM = distVal[1];
    dmsg.distR = distVal[2];
    ROS_INFO("L: %f\nM: %f\nR: %f\n", dmsg.distL, dmsg.distM, dmsg.distR);
    pub.publish(dmsg);
    ros::spinOnce();
    loop_rate.sleep();
  }
    return 0;
}


