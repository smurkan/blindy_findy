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
#include <vector>
#include <algorithm>
#include "blindy_findy/distances.h"
#include "blindy_findy/frame.h"

static const std::string OPENCV_WINDOW = "Image window";

ros::Publisher* pub;
ros::Publisher* pub2;
bool firstFrame = true;
int numberOfWindows = 10;


float returnSlope(float line[], int end, int start)
{
  std::vector<float> slopes;
  for (int i = start+1; i <= end; i++)
  {
    slopes.push_back(line[i] - line[i-1]);
    
  }
  std::sort(slopes.begin(), slopes.end());
  return slopes.at(slopes.size()/2);
}

int ReturnEndOfFloor(float line[], int windowSize, int height)
{
  double slope = 0;
  int index = windowSize;
  while (slope <= 0 && index >= 1)
  {
    index--;
    {
      slope = returnSlope(line, (index + 1) * (height - 1) / windowSize, (index * (height - 1)) / windowSize);
    }
  }
  index = (index) * (height - 1) / windowSize;
  return index;
}


float ReturnSmallestDistance(float line[], int index)
{
  float shortest = 666;
  for (int i = index; i >= numberOfWindows; i--)
  {
    if (line[i] < shortest && line[i] !=0)
      shortest = line[i];
  }
  return shortest;
}

void writeToFile(float line[], int height)
{
  std::ofstream output;
  output.open("10-05-open5.txt", std::ios::out | std::ios::app);
  for(int i=0;i<height;i++)
  {
    output << line[i] << ",";
    if(i == height-1)
    {
      output << 0 << "," << 0 << "," << 1;
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
  int no_of_lines = 10;
  float rightline[no_of_lines][height];
  float midline[no_of_lines][height];
  float leftline[no_of_lines][height];
  float distVal[3];
  std::vector<int> indexLList;
  std::vector<int> indexMList;
  std::vector<int> indexRList;
  int indexL;
  int indexM;
  int indexR;
  int lastIndexL = 0;
  int lastIndexM = 0;
  int lastIndexR = 0;
  float alpha = 1;

    
  //-------extracting pixel values-----------------------------------------
  float depthMap[height][width];
  for(int n=0;n<height;n++)
  {
    for(int m=0;m<width;m++)
    {
      cv::Scalar intensity = cv_ptr->image.at<float>(n,m);
      depthMap[n][m]= intensity.val[0];
      if(m==(width/4)- (no_of_lines/2))
        for(int i=0;i<no_of_lines; i++)
          {
            rightline[i][n] = depthMap[n][m+i];

          }
      if(m==(width/2)- (no_of_lines/2))
        for(int i=0;i<no_of_lines; i++)
          {
            midline[i][n] = depthMap[n][m+i];

          }
      if(m==(3*width/4)- (no_of_lines/2))
        for(int i=0;i<no_of_lines; i++)
          {
            leftline[i][n] = depthMap[n][m+i];

          }
    }
  }
  //pixel index of nearest object in each line
  for (int i=0; i < no_of_lines; i++)
  {   
  indexLList.push_back(ReturnEndOfFloor(leftline[i], numberOfWindows, height));
  indexMList.push_back(ReturnEndOfFloor(midline[i], numberOfWindows, height));
  indexRList.push_back(ReturnEndOfFloor(rightline[i], numberOfWindows, height));
  //distance values of those pixels
  //for data collecting, writes midline to txt file
  //writeToFile(midline, height);
  //produces viewable image
  }

  if (firstFrame)
  {
  lastIndexL = (indexLList.at(no_of_lines/2));
  lastIndexM = (indexMList.at(no_of_lines/2));
  lastIndexR = (indexRList.at(no_of_lines/2));
  firstFrame = false; 
  }

  std::sort(indexLList.begin(), indexLList.end());
  std::sort(indexMList.begin(), indexMList.end());
  std::sort(indexRList.begin(), indexRList.end());

  indexL = alpha * (indexLList.at(no_of_lines/2)) + (1 - alpha) * lastIndexL;
  indexM = alpha * (indexMList.at(no_of_lines/2)) + (1 - alpha) * lastIndexM;
  indexR = alpha * (indexRList.at(no_of_lines/2)) + (1 - alpha) * lastIndexR;

  lastIndexL = indexL;
  lastIndexM = indexM;
  lastIndexR = indexR;


  distVal[0] = ReturnSmallestDistance(leftline[(no_of_lines/2)+1], indexL);
  distVal[1] = ReturnSmallestDistance(midline[(no_of_lines/2)+1], indexM);
  distVal[2] = ReturnSmallestDistance(rightline[(no_of_lines/2)+1], indexR);
  
  
  double minVal, maxVal;
  minMaxLoc(cv_ptr->image, &minVal, &maxVal); //find minimum and maximum intensities
  //ROS_INFO("MAXVAL: %f\n MINVAL: %f\n", maxVal, minVal);
  cv::Mat blur_img;
  if(minVal < 0.7)
  {
    minVal=0;
  }
  if(maxVal>20)
  {
    maxVal=20;
  }
  cv_ptr->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
  //draws circles on all nearest distances
  /*cv::circle(blur_img, cv::Point((width/4), indexR), 10, cv::Scalar(255,0,0),2);
  cv::circle(blur_img, cv::Point((2*width/4), indexM), 10, cv::Scalar(255,0,0),2);
  cv::circle(blur_img, cv::Point((3*width/4), indexL), 10, cv::Scalar(255,0,0),2);
  cv::line(blur_img, cv::Point((width/4), 0), cv::Point((width/4), height), cv::Scalar(255,0,0), 2, 8);
  cv::line(blur_img, cv::Point((2*width/4), 0), cv::Point((2*width/4), height), cv::Scalar(255,0,0), 2, 8);
  cv::line(blur_img, cv::Point((3*width/4), 0), cv::Point((3*width/4), height), cv::Scalar(255,0,0), 2, 8);
  
  cv::imshow(OPENCV_WINDOW, blur_img);
  cv::waitKey(3);*/
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

  /*blindy_findy::frame fmsg;
  for(int i=0;i<376;i++)
  {
    fmsg.stairFrame[i] = midline[i];
  }
  ROS_INFO("frame[0]: %f\n", fmsg.stairFrame[0]);
  pub2->publish(fmsg);*/
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
  ros::Publisher publisher2 = nh.advertise<blindy_findy::frame>("frames", 1);
  pub2 = &publisher2;
  

  image_sub = it.subscribe("/depth/depth_registered", 1, Cbfunc);
  image_pub = it.advertise("/output_video", 1);
  cv::namedWindow(OPENCV_WINDOW);
  
  cv::destroyWindow(OPENCV_WINDOW);

  ros::spin();
  return 0;
}


