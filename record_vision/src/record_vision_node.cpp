#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <opencv2/opencv.hpp>//highgui/highgui.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>//highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <alice_msgs/FoundObjectArray.h>

#include "log_manager.h"

using namespace std;
using namespace boost::filesystem;
using namespace boost::lambda;

void ObjectCallback(const alice_msgs::FoundObjectArray &msg);
void ImageCallback(const sensor_msgs::ImageConstPtr &msg);

Log *log_file;
cv::Mat img_msg;
cv::VideoWriter *video_writer;

bool zed_on = false;

int main(int argc, char**argv)
{
  ros::init(argc, argv, "record_vision_node");
  ros::NodeHandle nh;

  ros::Subscriber sub_object = nh.subscribe("/heroehs/detected_objects", 1, ObjectCallback);
  ros::Subscriber sub_image = nh.subscribe("/heroehs/environment_detector/image", 1, ImageCallback);

  ros::Time t = ros::Time::now();

  string path = "/home/alice1-nuke/logs/robocup_2018/vision";
  string date = GetYYMMDD();
  string seconds = GetSeconds(t.sec);
  //nh.getParam("path", path);
  string log_path = path+"/"+date+"_vision_"+seconds+".txt";
  string video_path = path+"/"+date+"_video_"+seconds+".avi";

  log_file = new Log(log_path);

  video_writer = new cv::VideoWriter;
  video_writer->open(
      video_path.c_str(), 
      CV_FOURCC('M','J','P','G'), 
      30, cv::Size(672, 376));

  ros::Duration hz(0.03);

  while(ros::ok())
  {
    if(t <= ros::Time::now() && zed_on)
    {
      t = ros::Time::now()+hz;
      video_writer->write(img_msg);
    }
    ros::spinOnce();
    usleep(10);
  }
}


void ObjectCallback(const alice_msgs::FoundObjectArray &msg)
{
  for(int i=0 ; i<msg.length ; i++)
  {
    stringstream ss;
    ss << GetSeconds(ros::Time::now().sec) << "/"
       << msg.data[i].name << "/"
       << msg.data[i].pos.x << "/"
       << msg.data[i].pos.y << "/"
       << msg.data[i].pos.z << "/"
       << msg.data[i].roi.x_offset << "/"
       << msg.data[i].roi.y_offset << "/"
       << msg.data[i].roi.width << "/"
       << msg.data[i].roi.height << "/"
       << endl;
    log_file->Write(ss.str());
  }
}

void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  zed_on = true;
  img_msg = cv_bridge::toCvShare(msg, "bgr8")->image;
}

