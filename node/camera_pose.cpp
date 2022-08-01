#include "../third-party/apriltags/TagDetector.h"
#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <getopt.h>

#include "ros/ros.h"
#include <ros/package.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../third-party/apriltags/CameraUtil.h"

#define DEFAULT_TAG_FAMILY "Tag25h9"

typedef struct CamTestOptions
{
    CamTestOptions() : params(),
                       family_str(DEFAULT_TAG_FAMILY),
                       error_fraction(1),
                       device_num(0),
                       focal_length(648),
                       tag_size(0.06),
                       frame_width(640),
                       frame_height(360),
                       mirror_display(false)
    {
    }
    TagDetectorParams params;
    std::string family_str;
    double error_fraction;
    int device_num;
    double focal_length;
    double tag_size;
    int frame_width;
    int frame_height;
    bool mirror_display;
} CamTestOptions;

cv::Mat frame;

// 图像帧回调函数
void ColorImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cam_pose_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber color_sub = it.subscribe("camera/color/image_raw", 1, ColorImageCallback);
    // 定义tf2广播
    tf2_ros::TransformBroadcaster broadcaster;
    int count = 0, currentIndex = 1;
    CamTestOptions opts;
    // 从launch文件获取参数
    nh.getParam("tag_size", opts.tag_size);
    nh.getParam("tag_family", opts.family_str);
    nh.getParam("image_width", opts.frame_width);
    nh.getParam("image_height", opts.frame_height);

    TagFamily family(opts.family_str);

    if (opts.error_fraction >= 0 && opts.error_fraction <= 1)
    {
        family.setErrorRecoveryFraction(opts.error_fraction);
    }
    cv::Point2d opticalCenter;

    opticalCenter.x = opts.frame_width * 0.5;
    opticalCenter.y = opts.frame_height * 0.5;

    TagDetectorParams &params = opts.params;
    TagDetector detector(family, params);

    TagDetectionArray detections;

    int cvPose = 1;
    double totalTime = 0;
    int totalIterations = 0;
    double s = opts.tag_size;
    double ss = 0.5 * s;
    double sz = s;

    enum
    {
        npoints = 8,
        nedges = 12
    };

    cv::Point3d src[npoints] = {
        cv::Point3d(-ss, -ss, 0),
        cv::Point3d(ss, -ss, 0),
        cv::Point3d(ss, ss, 0),
        cv::Point3d(-ss, ss, 0),
        cv::Point3d(-ss, -ss, sz),
        cv::Point3d(ss, -ss, sz),
        cv::Point3d(ss, ss, sz),
        cv::Point3d(-ss, ss, sz),
    };
    //用于画立方体
    int edges[nedges][2] = {

        {0, 1},
        {1, 2},
        {2, 3},
        {3, 0},

        {4, 5},
        {5, 6},
        {6, 7},
        {7, 4},

        {0, 4},
        {1, 5},
        {2, 6},
        {3, 7}

    };

    cv::Point2d dst[npoints];

    double f = opts.focal_length;

    double K[9] = {
        f, 0, opticalCenter.x,
        0, f, opticalCenter.y,
        0, 0, 1};

    cv::Mat_<cv::Point3d> srcmat(npoints, 1, src);
    cv::Mat_<cv::Point2d> dstmat(npoints, 1, dst);

    cv::Mat_<double> Kmat(3, 3, K);

    cv::Mat_<double> distCoeffs = cv::Mat_<double>::zeros(4, 1);

    while (ros::ok())
    {
        ros::spinOnce();
        if (!frame.empty())
        {
            // 计算耗时
            double tick = (double)cv::getTickCount();
            detector.process(frame, opticalCenter, detections);
            double currentTime = ((double)cv::getTickCount() - tick) / cv::getTickFrequency();
            totalTime += currentTime;
            totalIterations++;
            if (totalIterations % 1 == 0)
            {
                std::cout << "Detection Time = " << currentTime * 1000 << " ms "
                          << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)"
                          << std::endl;
            }
            if (!detections.empty())
            {
                count++;

                for (size_t i = 0; i < detections.size(); ++i)
                {
                    std::cout << "i = " << i << " id = " << detections[i].id << " code = " << detections[i].code << std::endl;

                    cv::Mat r, t;

                    CameraUtil::homographyToPoseCV(f, f, s,
                                                   detections[i].homography,
                                                   r, t);
                    cv::Mat rot(3, 3, CV_64FC1);
                    cv::Mat Rvec64;
                    r.convertTo(Rvec64, CV_64FC1);
                    cv::Rodrigues(Rvec64, rot);
                    cv::Mat tran64;
                    t.convertTo(tran64, CV_64FC1);

                    tf2::Matrix3x3 tf_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2), rot.at<double>(1, 0),
                                          rot.at<double>(1, 1), rot.at<double>(1, 2), rot.at<double>(2, 0), rot.at<double>(2, 1),
                                          rot.at<double>(2, 2));

                    tf2::Vector3 tf_orig(tran64.at<double>(0, 0), tran64.at<double>(1, 0), tran64.at<double>(2, 0));

                    // tf坐标变换发布
                    tf2::Transform ts(tf_rot, tf_orig);
                    geometry_msgs::TransformStamped tfs;
                    tfs.header.frame_id = "camera";
                    tfs.header.stamp = ros::Time::now();
                    tfs.child_frame_id = "marker";
                    tfs.transform.rotation.x = ts.getRotation().x();
                    tfs.transform.rotation.y = ts.getRotation().y();
                    tfs.transform.rotation.z = ts.getRotation().z();
                    tfs.transform.rotation.w = ts.getRotation().w();
                    tfs.transform.translation.x = ts.getOrigin().x();
                    tfs.transform.translation.y = ts.getOrigin().y();
                    tfs.transform.translation.z = ts.getOrigin().z();
                    broadcaster.sendTransform(tfs);

                    cv::projectPoints(srcmat, r, t, Kmat, distCoeffs, dstmat);

                    // 画立方体
                    for (int j = 0; j < nedges; ++j)
                    {
                        cv::line(frame,
                                 dstmat(edges[j][0], 0),
                                 dstmat(edges[j][1], 0),
                                 cvPose ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0),
                                 1, cv::LINE_AA);
                    }
                }
            }

            cv::imshow("apirltag test", frame);
            // 按q或ESC退出
            if (cv::waitKey(1) == 27 || cv::waitKey(1) == 'q')
            {
                break;
            }
        }
    }

    detector.reportTimers();

    return 0;
}
