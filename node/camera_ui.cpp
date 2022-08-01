#define CVUI_IMPLEMENTATION
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "common/include/cvui.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#define WINDOW_NAME "CVUI"

cv::Mat depth_image, color_image, render_depth;
int image_width = 0, image_height = 0;

void ColorImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        if (image_width == 0)
        {
            image_width = image.size().width;
            image_height = image.size().height;
        }
        color_image = image;
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
        depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
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
        render_depth = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv)
{

    int depth_value = 0;
    bool enable_record = false;
    // 彩色图和深度图的鼠标位置
    int mouse_x = 0, mouse_y = 0;
    // 自定义图片的鼠标位置
    int custom_point_x = 0, custom_point_y = 0;
    // 彩色图，深度图，用户自定义图片1，用户自定义图片2
    cv::Mat custom_image1, custom_image2;
    cv::Mat custom_image(450, 800, CV_8UC3, cv::Scalar(255, 255, 0));
    // 警告图，如果要显示的图片为空，则显示警告
    cv::Mat empty_image(450, 800, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat back_ground(1000, 1850, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::putText(custom_image, "put your custom image here", cv::Point(150, 225), 3, 1, cv::Scalar(0, 0, 0), 2);
    cv::putText(empty_image, "empty, check the image source", cv::Point(130, 225), 3, 1, cv::Scalar(0, 0, 0), 2);

    // 订阅信息
    ros::init(argc, argv, "image_listener");
    ROS_INFO("Start subscribing \n");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber color_sub = it.subscribe("camera/color/image_raw", 1, ColorImageCallback);
    image_transport::Subscriber depth_sub = it.subscribe("camera/depth/image_rect_raw", 1, DepthImageCallback);
    image_transport::Subscriber rendered_depth_sub = it.subscribe("camera/depth/rendered_depth", 1, RenderedDepthCallback);

    //////////////////////////录像设置部分//////////////////////////////////
    cv::VideoWriter record;
    std::string record_path = ros::package::getPath("hikrobot_ros") + "/video";
    char VideoName[50];
    struct tm p;
    time_t time_;
    time(&time_);
    p = *(gmtime(&time_));
    sprintf(VideoName, "%d-%d-%d--%d-%d.avi", p.tm_year + 1900, p.tm_mon + 1, p.tm_mday, p.tm_hour + 8, p.tm_min);
    record_path = record_path + "/" + VideoName;
    int temp_width = 0, temp_height = 0;
    nh.getParam("image_width", temp_width);
    nh.getParam("image_height", temp_height);
    std::cout << temp_width << " " << temp_height << std::endl;
    std::cout << "record = " << record_path << std::endl;
    record.open(record_path.c_str(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(temp_width, temp_height));
    //FIXME: 相机运行需要root权限，在root权限下opencv录的视频是带锁的，建议新开终端运行此节点
    //////////////////////////////////////////////////////////////////////

    cvui::init(WINDOW_NAME);

    while (ros::ok())
    {
        cv::Mat show_color_image;
        cv::Mat show_render_depth;
        // 录像，可以将color_image替换成其他想要录制的frame
        if (enable_record)
        {
            record << color_image;
        }

        if (!color_image.empty())
        {
            show_color_image = color_image.clone();
            cv::resize(show_color_image, show_color_image, cv::Size(800, 450));
        }
        if (!render_depth.empty())
        {
            show_render_depth = render_depth.clone();
            cv::resize(show_render_depth, show_render_depth, cv::Size(800, 450));
        }

        // 显示title
        cvui::text(back_ground, 530, 10, "|color view|", 0.8, 0xFFFFFFFF);
        cvui::text(back_ground, 1350, 10, "|depth view|", 0.8, 0xFFFFFFFF);
        cvui::text(back_ground, 510, 500, "|custom1 view|", 0.8, 0xFFFFFFFF);
        cvui::text(back_ground, 1320, 500, "|custom2 view|", 0.8, 0xFFFFFFFF);

        //显示数值控件
        int pose_x = 10, pose_y = 10, text_pos_y = 60;
        int y_step = 90;
        cvui::window(back_ground, pose_x, pose_y, 180, 80, "image width", 0.8);
        cvui::text(back_ground, pose_x + 60, text_pos_y, std::to_string(image_width), 0.8, 0xFFFFFFFF);

        pose_y += y_step;
        text_pos_y += y_step;
        cvui::window(back_ground, pose_x, pose_y, 180, 80, "image height", 0.8);
        cvui::text(back_ground, pose_x + 60, text_pos_y, std::to_string(image_height), 0.8, 0xFFFFFFFF);

        pose_y += y_step;
        text_pos_y += y_step;
        cvui::window(back_ground, pose_x, pose_y, 180, 80, "mouse point", 0.8);
        cvui::text(back_ground, pose_x + 10, text_pos_y, "(" + std::to_string(mouse_x) + "," + std::to_string(mouse_y) + ")", 0.8, 0xFFFFFFFF);

        pose_y += y_step;
        text_pos_y += y_step;
        cvui::window(back_ground, pose_x, pose_y, 180, 80, "depth value", 0.8);
        cvui::text(back_ground, pose_x + 50, text_pos_y, std::to_string(depth_value), 0.8, 0xFFFFFFFF);

        pose_y += y_step;
        text_pos_y += y_step;
        cvui::window(back_ground, pose_x, pose_y, 180, 80, "custom mouse", 0.8);
        cvui::text(back_ground, pose_x + 10, text_pos_y, "(" + std::to_string(custom_point_x) + "," + std::to_string(custom_point_y) + ")", 0.8, 0xFFFFFFFF);

        /////////////////////显示部分///////////////////////
        // 显示彩色图
        if (!show_color_image.empty())
        {
            cvui::image(back_ground, 200, 40, show_color_image);

            // 这里作为演示，将彩色图进行二值化，用户可以对custom_image做任何图像处理
            cv::cvtColor(show_color_image, custom_image1, cv::COLOR_BGR2GRAY);
            cv::threshold(custom_image1, custom_image1, 150, 255, cv::THRESH_OTSU);
            //转换成伪彩色图显示灰度，因为源代码使用了copyTo
            cv::cvtColor(custom_image1, custom_image1, cv::COLOR_GRAY2BGR);
        }
        // 如果彩色图为空，则在该位置显示警告
        else
            cvui::image(back_ground, 200, 40, empty_image);

        // 显示深度图
        if (!show_render_depth.empty())
        {
            cvui::image(back_ground, 1020, 40, show_render_depth);
        }
        else
            cvui::image(back_ground, 1020, 40, empty_image);

        // 显示用户自定义图片1，可将custom_image1替换为要显示的Mat
        if (!custom_image1.empty())
        {
            cvui::image(back_ground, 200, 530, custom_image1);
        }
        else
            cvui::image(back_ground, 200, 530, empty_image);

        // 显示用户自定义图片2
        custom_image2 = custom_image;
        if (!custom_image2.empty())
        {
            cvui::image(back_ground, 1020, 530, custom_image);
        }
        else
            cvui::image(back_ground, 1020, 530, empty_image);

        //////////////////////////交互部分///////////////////////
        // 深度图鼠标交互区域
        int status_depth = cvui::iarea(1020, 40, 800, 450);
        if (status_depth == cvui::OVER)
        {
            mouse_x = (cvui::mouse().x - 1020) * image_width / 800;
            mouse_y = (cvui::mouse().y - 40) * image_height / 450;
            if (!depth_image.empty())
            {
                depth_value = static_cast<int>(depth_image.at<unsigned short>(cv::Point(mouse_x, mouse_y)));
            }
        }
        // 彩色图鼠标交互区域
        int status_color = cvui::iarea(200, 40, 800, 450);
        if (status_color == cvui::OVER)
        {
            mouse_x = (cvui::mouse().x - 200) * image_width / 800;
            mouse_y = (cvui::mouse().y - 40) * image_height / 450;
        }
        //自定义图片鼠标交互区域

        int status_custom1 = cvui::iarea(200, 530, 800, 450);
        if (status_custom1 == cvui::OVER)
        {
            custom_point_x = (cvui::mouse().x - 200) * (custom_image1.size().width) / 800;
            custom_point_y = (cvui::mouse().y - 530) * (custom_image1.size().height) / 450;
        }
        int status_custom2 = cvui::iarea(1020, 530, 800, 450);
        if (status_custom2 == cvui::OVER)
        {
            custom_point_x = (cvui::mouse().x - 1020) * (custom_image2.size().width) / 800;
            custom_point_y = (cvui::mouse().y - 530) * (custom_image2.size().height) / 450;
        }

        cvui::update();

        cv::imshow(WINDOW_NAME, back_ground);
        cv::waitKey(1);
        ////////////////////////// 按钮部分/////////////////////////////
        if (cvui::button(back_ground, 60, 800, "&Record"))
        {
            enable_record = !enable_record;
            if (enable_record)
            {
                std::cout << "start recording!" << std::endl;
            }
            else
            {
                std::cout << "end recording!" << std::endl;
            }
        }
        if (cvui::button(back_ground, 60, 900, "&EXIT"))
        {
            break;
        }
        ros::spinOnce();
    }
}
