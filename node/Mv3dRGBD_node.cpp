#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/PointCloud.h>
#include "std_msgs/String.h"
#include "common/common.hpp"
#include "common/RenderImage.hpp"

class MV3D_RGBD_Camera
{
public:
    MV3D_RGBD_Camera()
    {
        LOG("first plug-in takes time to initialize\n");
        ASSERT_OK(MV3D_RGBD_GetSDKVersion(&stVersion));
        ASSERT_OK(MV3D_RGBD_Initialize());
        unsigned int nDevNum = 0;
        ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB, &nDevNum));
        LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
        ASSERT(nDevNum);
        LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);

        // 查找设备
        std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
        ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &devs[0], nDevNum, &nDevNum));
        unsigned int nIndex = 0;

        ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
        LOGD("OpenDevice success \n");
        for (unsigned int i = 0; i < nDevNum; i++)
        {
            LOG("Device [%d]\n", i + 1);
            print_camera_info(handle, devs[nDevNum], &stRgbCalibInfo, &stDepthCalibInfo);
        }
    }

    ~MV3D_RGBD_Camera()
    {
        ASSERT_OK(MV3D_RGBD_Stop(handle));
        ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
        ASSERT_OK(MV3D_RGBD_Release());
    }

    void GetParm()
    {
        nh.getParam("image_width", Image_Width);
        nh.getParam("image_height", Image_Height);
        nh.getParam("exposure", Exposure);
        nh.getParam("gain", Gain);
        nh.getParam("enable_laser", Enable_Laser);
        nh.getParam("enable_color", Enable_Color);
        nh.getParam("enable_depth", Enable_Depth);
        nh.getParam("enable_infra", Enable_Infra);
        nh.getParam("enable_pointcloud", Enable_PointCloud);
        nh.getParam("color_info", Color_CameraUrl);
        nh.getParam("depth_info", Depth_CameraUrl);
    }

    void ParamInitialize()
    {
        setParam(handle, MV3D_RGBD_BOOL_LASERENABLE, Enable_Laser);
        // setParam(handle, MV3D_RGBD_FLOAT_EXPOSURETIME, Exposure);
        // setParam(handle, MV3D_RGBD_FLOAT_GAIN, Gain);
        if (Image_Width == 640 && Image_Height == 360)
        {
            setParam(handle, MV3D_RGBD_ENUM_RESOLUTION, Mv3dRgbdResolutionType::Mv3dRgbdResolution_640_360);
        }
        else
        {
            setParam(handle, MV3D_RGBD_ENUM_RESOLUTION, Mv3dRgbdResolutionType::Mv3dRgbdResolution_1280_720);
        }

        color_camera_info =
            boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh, "color", Color_CameraUrl));

        depth_camera_info = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh, "depth", Depth_CameraUrl));

        color_info = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(color_camera_info->getCameraInfo()));

        depth_info = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo(depth_camera_info->getCameraInfo()));

        RgbCameraMatrix.at<double>(0, 0) = stRgbCalibInfo.stIntrinsic.fData[0];
        RgbCameraMatrix.at<double>(0, 2) = stRgbCalibInfo.stIntrinsic.fData[2];
        RgbCameraMatrix.at<double>(1, 1) = stRgbCalibInfo.stIntrinsic.fData[4];
        RgbCameraMatrix.at<double>(1, 2) = stRgbCalibInfo.stIntrinsic.fData[5];
        RgbCameraMatrix.at<double>(2, 2) = 1;

        RgbDistCoeffs.at<double>(0) = stRgbCalibInfo.stDistortion.fData[0];
        RgbDistCoeffs.at<double>(1) = stRgbCalibInfo.stDistortion.fData[1];
        RgbDistCoeffs.at<double>(2) = stRgbCalibInfo.stDistortion.fData[2];
        RgbDistCoeffs.at<double>(3) = stRgbCalibInfo.stDistortion.fData[3];

        DepthCameraMatrix.at<double>(0, 0) = stDepthCalibInfo.stIntrinsic.fData[0];
        DepthCameraMatrix.at<double>(0, 2) = stDepthCalibInfo.stIntrinsic.fData[2];
        DepthCameraMatrix.at<double>(1, 1) = stDepthCalibInfo.stIntrinsic.fData[4];
        DepthCameraMatrix.at<double>(1, 2) = stDepthCalibInfo.stIntrinsic.fData[5];
        DepthCameraMatrix.at<double>(2, 2) = 1;

        DepthDistCoeffs.at<double>(0) = stDepthCalibInfo.stDistortion.fData[0];
        DepthDistCoeffs.at<double>(1) = stDepthCalibInfo.stDistortion.fData[1];
        DepthDistCoeffs.at<double>(2) = stDepthCalibInfo.stDistortion.fData[2];
        DepthDistCoeffs.at<double>(3) = stDepthCalibInfo.stDistortion.fData[3];
    }

    void RosPublish_Initialize()
    {
        if (Enable_Color)
        {
            color_pub = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw", 10);
            pub_color_info = nh.advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info", 30);
        }
        if (Enable_Depth)
        {
            depth_pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_rect_raw", 10);
            rendered_depth_pub = nh.advertise<sensor_msgs::Image>("/camera/depth/rendered_depth", 10);
            pub_depth_info = nh.advertise<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 30);
        }
        if (Enable_Infra)
        {
            infra_left_pub = nh.advertise<sensor_msgs::Image>("/camera/infra_left/image_rect_raw", 10);
            infra_right_pub = nh.advertise<sensor_msgs::Image>("/camera/infra_right/image_rect_raw", 10);
        }
        if (Enable_PointCloud)
            cloud_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud", 50);
    }

    void Publish_Message()
    {
        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        if (nRet == MV3D_RGBD_OK)
        {
            RIFrameInfo depth = {0};
            RIFrameInfo rgb = {0};
            parseFrame(&stFrameData, &depth, &rgb);

            RIFrameInfo rendered_depth = depth;
            AUTO_MALLOC_DATA_INFO ConvertData;
            ConvertData.ResizeData((depth.nWidth + 3) * depth.nHeight * 3);
            rendered_depth.enPixelType = RIPixelType_RGB8_Packed;
            rendered_depth.pData = ConvertData.pData;
            RenderImage::ColorTableType render_color = ColorTableWarm;

            RenderImage::ConvertC16_2_RGB(depth.pData, depth.nWidth, depth.nHeight, rendered_depth.pData, ColorTable::GetColorTable(ColorTableWarm));

            color_frame = cv::Mat(rgb.nHeight, rgb.nWidth, CV_8UC3, rgb.pData);
            depth_frame = cv::Mat(depth.nHeight, depth.nWidth, CV_16UC1, depth.pData);
            cv::imwrite("/home/tdt/1.png", color_frame);
            cv::imwrite("/home/tdt/1.pgm", depth_frame);
            rendered_depth_frame = cv::Mat(rendered_depth.nHeight, rendered_depth.nWidth, CV_8UC3, rendered_depth.pData);

            if (!color_frame.empty() && Enable_Color)
            {
                color_info->header.frame_id = "/color_frame";
                color_info->header.stamp = ros::Time::now();
                color_info->width = color_frame.size().width;
                color_info->height = color_frame.size().height;

                cv::Mat Projection_Matrix;
                if (color_info->width == 1280)
                {
                    Projection_Matrix = cv::getOptimalNewCameraMatrix(RgbCameraMatrix, RgbDistCoeffs, cv::Size(1280, 720), 0);
                }
                else
                {
                    Projection_Matrix = cv::getOptimalNewCameraMatrix(RgbCameraMatrix, RgbDistCoeffs, cv::Size(640, 360), 0);
                }
                boost::array<double, 12> P = {
                    Projection_Matrix.at<double>(0, 0), Projection_Matrix.at<double>(0, 1), Projection_Matrix.at<double>(0, 2), Projection_Matrix.at<double>(0, 3),
                    Projection_Matrix.at<double>(1, 0), Projection_Matrix.at<double>(1, 1), Projection_Matrix.at<double>(1, 2), Projection_Matrix.at<double>(1, 3),
                    Projection_Matrix.at<double>(2, 0), Projection_Matrix.at<double>(2, 1), Projection_Matrix.at<double>(2, 2), Projection_Matrix.at<double>(2, 3)};
                color_info->P = P;

                pub_color_info.publish(color_info);

                color_ptr->image = color_frame;
                sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", color_frame).toImageMsg();
                color_msg->header.stamp = ros::Time::now();
                color_msg->header.frame_id = "Mv3dRGBD_color";
                color_pub.publish(color_msg);
            }
            if (!depth_frame.empty() && Enable_Depth)
            {
                depth_info->header.frame_id = "/depth_frame";
                depth_info->header.stamp = ros::Time::now();
                depth_info->width = depth_frame.size().width;
                depth_info->height = depth_frame.size().height;
                cv::Mat Projection_Matrix;
                if (depth_info->width == 1280)
                {
                    Projection_Matrix = cv::getOptimalNewCameraMatrix(DepthCameraMatrix, DepthDistCoeffs, cv::Size(1280, 720), 0);
                }
                else
                {
                    Projection_Matrix = cv::getOptimalNewCameraMatrix(DepthCameraMatrix, DepthDistCoeffs, cv::Size(640, 360), 0);
                }
                boost::array<double, 12> P = {
                    Projection_Matrix.at<double>(0, 0), Projection_Matrix.at<double>(0, 1), Projection_Matrix.at<double>(0, 2), Projection_Matrix.at<double>(0, 3),
                    Projection_Matrix.at<double>(1, 0), Projection_Matrix.at<double>(1, 1), Projection_Matrix.at<double>(1, 2), Projection_Matrix.at<double>(1, 3),
                    Projection_Matrix.at<double>(2, 0), Projection_Matrix.at<double>(2, 1), Projection_Matrix.at<double>(2, 2), Projection_Matrix.at<double>(2, 3)};
                depth_info->P = P;
                pub_depth_info.publish(depth_info);

                depth_ptr->image = depth_frame;
                sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_frame).toImageMsg();
                depth_msg->header.stamp = ros::Time::now();
                depth_msg->header.frame_id = "Mv3dRGBD_depth";
                depth_pub.publish(depth_msg);
            }
            if (!rendered_depth_frame.empty() && Enable_Depth)
            {
                render_depth_ptr->image = rendered_depth_frame;
                sensor_msgs::ImagePtr render_depth_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rendered_depth_frame).toImageMsg();
                render_depth_msg->header.stamp = ros::Time::now();
                render_depth_msg->header.frame_id = "Mv3dRGBD_RenderDepth";
                rendered_depth_pub.publish(render_depth_msg);
            }
            for (int i = 0; i < stFrameData.nImageCount; i++)
            {
                if (Enable_Infra)
                {
                    cv::Mat left_ir(stFrameData.stImageData[2].nHeight, stFrameData.stImageData[2].nWidth, CV_8UC1, stFrameData.stImageData[2].pData);
                    sensor_msgs::ImagePtr InfraLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", left_ir).toImageMsg();
                    InfraLeftMsg->header.stamp = ros::Time::now();
                    InfraLeftMsg->header.frame_id = "Mv3dRGBD_InfraLeft";
                    infra_left_pub.publish(InfraLeftMsg);
                }
                if (Enable_Infra)
                {
                    cv::Mat right_ir(stFrameData.stImageData[3].nHeight, stFrameData.stImageData[3].nWidth, CV_8UC1, stFrameData.stImageData[3].pData);
                    sensor_msgs::ImagePtr InfraRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", right_ir).toImageMsg();
                    InfraRightMsg->header.stamp = ros::Time::now();
                    InfraRightMsg->header.frame_id = "Mv3dRGBD_InfraRight";
                    infra_right_pub.publish(InfraRightMsg);
                }

                if (Enable_PointCloud && (ImageType_Depth == stFrameData.stImageData[i].enImageType))
                {
                    MV3D_RGBD_IMAGE_DATA stPointCloudImage;
                    int nRet = MV3D_RGBD_MapDepthToPointCloud(handle, &stFrameData.stImageData[i], &stPointCloudImage);
                    if (MV3D_RGBD_OK != nRet)
                    {
                        break;
                    }
                    if (nullptr == stPointCloudImage.pData)
                    {
                        LOG("error empty data\n");
                        continue;
                    }

                    int PointNum = stPointCloudImage.nDataLen / (sizeof(float) * 3);
                    cloud.header.frame_id = "pointcloud_frame";
                    cloud.points.resize(PointNum);
                    cloud.channels.resize(1);
                    cloud.channels[0].name = "intensities";
                    cloud.channels[0].values.resize(PointNum);
                    cloud.header.stamp = ros::Time::now();

                    float *pSrcValue = (float *)stPointCloudImage.pData;
                    for (int nPntIndex = 0; nPntIndex < PointNum; ++nPntIndex)
                    {
                        cloud.points[nPntIndex].x = pSrcValue[nPntIndex * 3 + 0] / 100;
                        cloud.points[nPntIndex].y = pSrcValue[nPntIndex * 3 + 1] / 100;
                        cloud.points[nPntIndex].z = pSrcValue[nPntIndex * 3 + 2] / 100;
                    }

                    cloud_pub.publish(cloud);
                }
            }
        }
    }

    void Initialize()
    {
        GetParm();
        ParamInitialize();
        ASSERT_OK(MV3D_RGBD_Start(handle));
        LOGD("Start work success");
        RosPublish_Initialize();
    }

private:
    cv::Mat RgbCameraMatrix = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);
    cv::Mat RgbDistCoeffs = cv::Mat(cv::Size(1, 4), CV_64FC1);

    cv::Mat DepthCameraMatrix = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);
    cv::Mat DepthDistCoeffs = cv::Mat(cv::Size(1, 4), CV_64FC1);

    int Image_Width = 1280;
    int Image_Height = 720;
    float Exposure = 1500;
    float Gain = 10;
    bool Enable_Laser = true;
    bool Enable_Color = true;
    bool Enable_Depth = true;
    bool Enable_Infra = false;
    bool Enable_PointCloud = false;
    string Color_CameraUrl, Depth_CameraUrl;

    ros::NodeHandle nh;
    ros::Publisher color_pub;
    ros::Publisher depth_pub;
    ros::Publisher infra_left_pub;
    ros::Publisher infra_right_pub;
    ros::Publisher rendered_depth_pub;
    ros::Publisher pub_color_info;
    ros::Publisher pub_depth_info;
    ros::Publisher cloud_pub;

    sensor_msgs::CameraInfoPtr color_info;
    sensor_msgs::CameraInfoPtr depth_info;
    sensor_msgs::PointCloud cloud;
    cv::Mat color_frame, depth_frame, rendered_depth_frame;

    cv_bridge::CvImagePtr color_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_bridge::CvImagePtr depth_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_bridge::CvImagePtr render_depth_ptr = boost::make_shared<cv_bridge::CvImage>();

    boost::shared_ptr<camera_info_manager::CameraInfoManager> color_camera_info;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> depth_camera_info;

    void *handle = NULL;
    MV3D_RGBD_VERSION_INFO stVersion;
    MV3D_RGBD_CALIB_INFO stRgbCalibInfo = {0};
    MV3D_RGBD_CALIB_INFO stDepthCalibInfo = {0};
    MV3D_RGBD_FRAME_DATA stFrameData = {0};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mv3dRgbd_node");

    MV3D_RGBD_Camera camera;
    camera.Initialize();

    while (ros::ok())
    {
        ros::spinOnce();
        camera.Publish_Message();
    }

    LOGD("Main done!");
    return 0;
}
