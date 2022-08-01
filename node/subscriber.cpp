#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat color;
void ColorImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    color = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("color view", color);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void DepthImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    // std::cout << static_cast<int>(image.at<unsigned short>(cv::Point(1000, 200))) << std::endl;
    cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    cv::imshow("depth view", image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void InfraLeftImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
    cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    cv::imshow("InfraLeft view", image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void InfraRightImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
    cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    cv::imshow("InfraRight view", image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void RenderedDepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("render depth view", image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "image_listener");
  ROS_INFO("Start subscribing \n");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 1, ColorImageCallback);
  image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image_rect_raw", 1, DepthImageCallback);
  image_transport::Subscriber infra_left_sub = it.subscribe("/camera/infra_left/image_rect_raw", 1, InfraLeftImageCallback);
  image_transport::Subscriber infra_right_sub = it.subscribe("/camera/infra_right/image_rect_raw", 1, InfraRightImageCallback);
  image_transport::Subscriber rendered_depth_sub = it.subscribe("/camera/depth/rendered_depth", 1, RenderedDepthCallback);
  while (ros::ok())
  {
    ros::spinOnce();
  }
}
