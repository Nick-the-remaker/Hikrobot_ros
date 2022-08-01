#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;

cv::Mat rgb, depth;

void ColorImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
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
        depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    // 相机内参，可从/camera/depth/camera_info话题中获取
    double camera_factor = 1000;
    double cx = 601.53;
    double cy = 370.16;
    double fx = 648.08;
    double fy = 648.08;

    ros::init(argc, argv, "aligned_depth_cloud");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("aligned_point_cloud", 1);
    image_transport::Subscriber color_sub = it.subscribe("camera/color/image_raw", 1, ColorImageCallback);
    image_transport::Subscriber depth_sub = it.subscribe("camera/depth/image_rect_raw", 1, DepthImageCallback);
    ros::Rate rate(3);
    sensor_msgs::PointCloud2 ros_point_cloud;

    while (ros::ok())
    {
        ros::spinOnce();

        if (!rgb.empty() && !depth.empty())
        {
            PointCloud::Ptr cloud(new PointCloud);
            // 遍历深度图
            for (int i = 0; i < depth.rows; i++)
                for (int j = 0; j < depth.cols; j++)
                {
                    ushort d = depth.ptr<ushort>(i)[j];
                    if (d == 0)
                        continue;
                    PointT p;
                    // 算出点的空间坐标
                    p.z = double(d) / camera_factor;
                    p.x = (j - cx) * p.z / fx;
                    p.y = -((i - cy) * p.z / fy);
                    // 提取彩色图对应点的RGB
                    p.b = rgb.ptr<uchar>(i)[j * 3];
                    p.g = rgb.ptr<uchar>(i)[j * 3 + 1];
                    p.r = rgb.ptr<uchar>(i)[j * 3 + 2];
                    cloud->points.push_back(p);
                }
            cloud->height = 1;
            cloud->width = cloud->points.size();
            cloud->is_dense = true;

            // 发布点云话题
            pcl::toROSMsg(*cloud, ros_point_cloud);
            ros_point_cloud.header.frame_id = "image_point_cloud";
            ros_point_cloud.header.stamp = ros::Time::now();
            pub.publish(ros_point_cloud);
            rate.sleep();
        }
    }

    return 0;
}
