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
//#include "blindy_findy/frame.h"

//name of output video window
static const std::string OPENCV_WINDOW = "Image window";

//publishers
ros::Publisher* pub;
ros::Publisher* pub2;
//used for check
bool firstFrame = true;
//window size for returnEndOfFloor function 
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

//for writing line values to a file, used for stairnet
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

//callback function
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

  // Calculate the estimate end of the floor for the number of lines identified by no_of_lines
  for (int i=0; i < no_of_lines; i++)
  {   
  indexLList.push_back(ReturnEndOfFloor(leftline[i], numberOfWindows, height));
  indexMList.push_back(ReturnEndOfFloor(midline[i], numberOfWindows, height));
  indexRList.push_back(ReturnEndOfFloor(rightline[i], numberOfWindows, height));

  }

// Initialze filter values
  if (firstFrame)
  {
  lastIndexL = (indexLList.at(no_of_lines/2));
  lastIndexM = (indexMList.at(no_of_lines/2));
  lastIndexR = (indexRList.at(no_of_lines/2));
  firstFrame = false; 
  }

// Sort the estiamted floor ends of lines for all three directions in order to apply a median filter
  std::sort(indexLList.begin(), indexLList.end());
  std::sort(indexMList.begin(), indexMList.end());
  std::sort(indexRList.begin(), indexRList.end());

// Weighted average filter applied to the median value of floor end estimates
  indexL = alpha * (indexLList.at(no_of_lines/2)) + (1 - alpha) * lastIndexL;
  indexM = alpha * (indexMList.at(no_of_lines/2)) + (1 - alpha) * lastIndexM;
  indexR = alpha * (indexRList.at(no_of_lines/2)) + (1 - alpha) * lastIndexR;

// Value of last measurment is saved for filtering
  lastIndexL = indexL;
  lastIndexM = indexM;
  lastIndexR = indexR;


  distVal[0] = ReturnSmallestDistance(leftline[(no_of_lines/2)+1], indexL);
  distVal[1] = ReturnSmallestDistance(midline[(no_of_lines/2)+1], indexM);
  distVal[2] = ReturnSmallestDistance(rightline[(no_of_lines/2)+1], indexR);

  //for data collecting to stairNet, writes midline to txt file
  //writeToFile(midline, height);

  //produces viewable image
  double minVal, maxVal;
  minMaxLoc(cv_ptr->image, &minVal, &maxVal); //find minimum and maximum intensities
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
  //draws circles on all nearest distances and visualizes the vertical lines
  cv::circle(blur_img, cv::Point((width/4), indexR), 10, cv::Scalar(255,0,0),2);
  cv::circle(blur_img, cv::Point((2*width/4), indexM), 10, cv::Scalar(255,0,0),2);
  cv::circle(blur_img, cv::Point((3*width/4), indexL), 10, cv::Scalar(255,0,0),2);
  cv::line(blur_img, cv::Point((width/4), 0), cv::Point((width/4), height), cv::Scalar(255,0,0), 2, 8);
  cv::line(blur_img, cv::Point((2*width/4), 0), cv::Point((2*width/4), height), cv::Scalar(255,0,0), 2, 8);
  cv::line(blur_img, cv::Point((3*width/4), 0), cv::Point((3*width/4), height), cv::Scalar(255,0,0), 2, 8);
  //displays the output video window 
  cv::imshow(OPENCV_WINDOW, blur_img);
  cv::waitKey(3);
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
  //prints out the distances from every line that is to be published
  ROS_INFO("L: %f\nM: %f\nR: %f\n", dmsg.distL, dmsg.distM, dmsg.distR);
  //published the message on the topic defined earlier
  pub->publish(dmsg);
  //for sending frames to stairNet
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
  //ros node name
  ros::init(argc, argv, "blindy_findy");
  //ros handle and image transport handles for video streams
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  //a ros nodehandle that publishes message type "distances" to a topic named distances
  ros::Publisher publisher = nh.advertise<blindy_findy::distances>("distances", 1);
  pub = &publisher;
  //a ros nodehandle that publishes message type "frame" to a topic named frames (only used for stairnet)
  //ros::Publisher publisher2 = nh.advertise<blindy_findy::frame>("frames", 1);
  //pub2 = &publisher2;
  
  //an image transport ros nodehandle that subscribes to the topic "/depth/depth_registered" and launches callback function "Cbfunc"
  image_sub = it.subscribe("/depth/depth_registered", 1, Cbfunc);
  //an image transport ros nodehandle that publishes to the topic "/output_video"
  image_pub = it.advertise("/output_video", 1);
  //output video window
  cv::namedWindow(OPENCV_WINDOW);
  //destructor
  cv::destroyWindow(OPENCV_WINDOW);
  //calls the callback function "Cbfunc" repeatedly as long as the topic subscribed to exists
  ros::spin();

  return 0;
}



