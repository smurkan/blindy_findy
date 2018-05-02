
#include <ros/ros.h>

#include "blindy_findy/distances.h"


/*int ReturnEndOfFloor(float line[], int windowSize, int height)
  {

     double slope = 0;
     int index = windowSize;
     while (slope <= 0 && index >= 1)
     {
       index--;
       {
         slope = line[(index + 1) * (height - 1) / windowSize] - line[(index * height) / windowSize];
       }
     }
     index = (index + 1) * (height - 1) / windowSize;
     return index;
   }

float ReturnSmallestDistance(float line[], int index)
  {
     float shortest = 666;
     for (int i = index; i >= 50; i--)
     {
        if (line[i] < shortest && line[i] !=0)
           shortest = line[i];
     }
     return shortest;
   }*/

/*void republish()
    {
      //VGA setting in ZED parameters
      int height = 376;
      int width = 672;
  
      float rightline[height];
      float midline[height];
      float leftline[height];
      int numberOfLines = 3;
      //float lines[numberOfLines][height];
      bool firstFrame = true;
      float distVal[3];

      cv_bridge::CvImagePtr localptr = cv_ptr;
    
      //-------put code here-----------------------------------------
      float depthMap[height][width];
      for(int n=0;n<height;n++)
      {
        for(int m=0;m<width;m++)
        {
          cv::Scalar intensity = localptr->image.at<float>(n,m);
          depthMap[n][m]= intensity.val[0];
          if(m==(height/4))
            rightline[n] = depthMap[n][m];
          if(m==(2*height/4))
            midline[n] = depthMap[n][m];
          if(m==(3*height/4))
            leftline[n] = depthMap[n][m];
        }
      }

      cv::line(localptr->image, cv::Point(width/4,0),cv::Point(width/4,height),
   cv::Scalar( 0, 0, 0 ),
   10,
   cv::LINE_8);
      cv::line(localptr->image, cv::Point(2*width/4,0),cv::Point(2*width/4,height),
   cv::Scalar( 0, 0, 0 ),
   10,
   cv::LINE_8);
      cv::line(localptr->image, cv::Point(3*width/4,0),cv::Point(3*width/4,height),
   cv::Scalar( 0, 0, 0 ),
   10,
   cv::LINE_8);

      distVal[0] = ReturnSmallestDistance(midline, ReturnEndOfFloor(midline, 10, height));
      distVal[1] = ReturnSmallestDistance(midline, ReturnEndOfFloor(midline, 10, height));
      distVal[2] = ReturnSmallestDistance(midline, ReturnEndOfFloor(midline, 10, height));
      printf("VAL: %f\n", distVal[0]);
      //-------------------------------------------------------------
      //blindy_findy::distances dmsg;
        //dmsg.distances.clear();
      //dmsg.distances.x = distVal[0];
        //dmsg.distances.y = distVal[1];
        //dmsg.distances.z = distVal[2];
      //ROS_INFO("%f", dmsg.distances.x);
      //pub->publish(dmsg);
  }


void Cbfunc(const sensor_msgs::ImageConstPtr& msg)
{
  int height = 376;
  int width = 672;

  float rightline[height];
  float midline[height];
  float leftline[height];
  
  int numberOfLines = 3;

    
  //-------put code here-----------------------------------------

  distVal[0] = ReturnSmallestDistance(leftline, ReturnEndOfFloor(leftline, 10, height));
  distVal[1] = ReturnSmallestDistance(midline, ReturnEndOfFloor(midline, 10, height));
  distVal[2] = ReturnSmallestDistance(rightline, ReturnEndOfFloor(rightline, 10, height));
  //printf("VAL: %f\n", distVal[0]);
  //put data in msg and publish-----------------------------------------------------
  blindy_findy::distances dmsg;
  dmsg.distL = distVal[0];
  dmsg.distM = distVal[1];
  dmsg.distR = distVal[2];
  //ROS_INFO("L: %f\nM: %f\nR: %f\n", dmsg.distL, dmsg.distM, dmsg.distR);
  pub->publish(dmsg);
}*/


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
  }
    return 0;
}


