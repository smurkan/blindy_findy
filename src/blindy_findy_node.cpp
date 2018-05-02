#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "blindy_findy/distances.h"



ros::Publisher* pub;

int ReturnEndOfFloor(float line[], int windowSize, int height)
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
   }

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
  }*/


void Cbfunc(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  int height = 376;
  int width = 672;

  float rightline[height];
  float midline[height];
  float leftline[height];
  float distVal[3];
  int numberOfLines = 3;

    
  //-------put code here-----------------------------------------
  float depthMap[height][width];
  for(int n=0;n<height;n++)
  {
    for(int m=0;m<width;m++)
    {
      cv::Scalar intensity = cv_ptr->image.at<float>(n,m);
      depthMap[n][m]= intensity.val[0];
      if(m==(height/4))
         rightline[n] = depthMap[n][m];
      if(m==(2*height/4))
         midline[n] = depthMap[n][m];
      if(m==(3*height/4))
        leftline[n] = depthMap[n][m];
    }
  }

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
}








int main(int argc, char** argv)
{
  ros::init(argc, argv, "blindy_findyc");
  //blindy_findyc bf;
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  //image_transport::Publisher image_pub;
  //std::string depthTopicName;

  //nh.param("publishers/distances_data/topic", publisherTopicName, std::string("/zed/depth/depth_registered"));
  
  ros::Publisher publisher = nh.advertise<blindy_findy::distances>("distances", 1);
  pub = &publisher;
  image_sub = it.subscribe("/camera/image_raw", 1, Cbfunc);

  //republish();
  
  

  ros::spin();
  return 0;
}


