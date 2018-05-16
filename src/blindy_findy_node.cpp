#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <fstream>
#include <iostream>
#include "blindy_findy/distances.h"

static const std::string OPENCV_WINDOW = "Image window";

ros::Publisher* pub;
bool firstFrame = true;

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

void writeToFile(float line[], int height)
{
  std::ofstream output;
  output.open("mid_values.txt", std::ios::out | std::ios::app);
  for(int i=0;i<height;i++)
  {
    output << line[i];
    if(i != height-1)
    {
      output << ",";
    }
  }
  output << std::endl;   
  output.close();
}

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
  int indexL;
  int indexM;
  int indexR;

    
  //-------extracting pixel values-----------------------------------------
  float depthMap[height][width];
  for(int n=0;n<height;n++)
  {
    for(int m=0;m<width;m++)
    {
      cv::Scalar intensity = cv_ptr->image.at<float>(n,m);
      depthMap[n][m]= intensity.val[0];
      if(m==(width/4))
         rightline[n] = depthMap[n][m];
      if(m==(2*width/4))
         midline[n] = depthMap[n][m];
      if(m==(3*width/4))
        leftline[n] = depthMap[n][m];
    }
  }
  //pixel index of nearest object in each line
  indexL = ReturnEndOfFloor(leftline, 50, height);
  indexM = ReturnEndOfFloor(midline, 50, height);
  indexR = ReturnEndOfFloor(rightline, 50, height);
  //distance values of those pixels
  distVal[0] = ReturnSmallestDistance(leftline,indexL);
  distVal[1] = ReturnSmallestDistance(midline,indexM);
  distVal[2] = ReturnSmallestDistance(rightline,indexR);
  /*
  //for data collecting, writes midline to txt file
  writeToFile(midline, height);
  //produces viewable image
  double minVal, maxVal;
  minMaxLoc(cv_ptr->image, &minVal, &maxVal); //find minimum and maximum intensities
  cv::Mat blur_img;
  cv_ptr->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
  //draws circles on all nearest distances
  cv::circle(blur_img, cv::Point((2*width/4), indexM), 10, CV_RGB(255,0,0));
  cv::circle(blur_img, cv::Point((3*width/4), indexL), 10, CV_RGB(255,0,0));
  cv::circle(blur_img, cv::Point((width/4), indexR), 10, CV_RGB(255,0,0));
  cv::imshow(OPENCV_WINDOW, blur_img);
  cv::waitKey(3);
  */
  //put data in msg and publish-----------------------------------------------------
  blindy_findy::distances dmsg;
  dmsg.distL = distVal[0];
  dmsg.distM = distVal[1];
  dmsg.distR = distVal[2];
  dmsg.pixL[0] = (3*width/4);
  dmsg.pixM[0] = (2*width/4);
  dmsg.pixR[0] = (width/4);
  dmsg.pixL[1] =  indexL;
  dmsg.pixM[1] =  indexM;
  dmsg.pixR[1] =  indexR;

  ROS_INFO("L: %f\nM: %f\nR: %f\n", dmsg.distL, dmsg.distM, dmsg.distR);
  pub->publish(dmsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blindy_findy");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  
  ros::Publisher publisher = nh.advertise<blindy_findy::distances>("distances", 1);
  pub = &publisher;
  image_sub = it.subscribe("/depth/depth_registered", 1, Cbfunc);

  image_pub = it.advertise("/output_video", 1);
  cv::namedWindow(OPENCV_WINDOW);
  
  cv::destroyWindow(OPENCV_WINDOW);

  ros::spin();
  return 0;
}


