/***************************************************************************************************
*
* 版权信息：版权所有 (c) 2022, 杭州海康机器人技术有限公司, 保留所有权利

*   @file       Mv3dRgbdDefine.h
*   @note       HangZhou Hikrobot Technology Co., Ltd. All Right Reserved.
*   @brief      RGBD Camera macro define
*
*   @date       2022/01/06
*   @note       V1.0.0 
*   
*   @warning    版权所有
****************************************************************************************************/


#ifndef _MV3D_RGBD_DEFINE_H_
#define _MV3D_RGBD_DEFINE_H_

#ifndef MV3D_RGBD_API
    #if (defined (_WIN32) || defined(WIN64))
        #if defined(MV3D_RGBD_EXPORTS)
            #define MV3D_RGBD_API __declspec(dllexport)
        #else
            #define MV3D_RGBD_API __declspec(dllimport)
        #endif
    #else
        #ifndef __stdcall
            #define __stdcall
        #endif

        #ifndef MV3D_RGBD_API
            #define  MV3D_RGBD_API
        #endif
    #endif
#endif

// ch:跨平台定义 En: Cross Platform Definition
#ifdef WIN32
    typedef signed char                     int8_t;
    typedef short                           int16_t;
    typedef int                             int32_t;
    typedef long long int                   int64_t;
    typedef unsigned char                   uint8_t;
    typedef unsigned short                  uint16_t;
    typedef unsigned int                    uint32_t;
    typedef unsigned long long              uint64_t;
    
    #define MV3D_RGBD_UNDEFINED             0xFFFFFFFF
#else
    #include <stdint.h>

    #define MV3D_RGBD_UNDEFINED             -1
#endif

/*****************************************状态码**************************************************/
// ch:正确码定义
#define MV3D_RGBD_OK                        0x00000000              // 成功，无错误

//通用错误码定义:范围0x80060000-0x800600FF
#define MV3D_RGBD_E_HANDLE                  0x80060000              // 错误或无效的句柄
#define MV3D_RGBD_E_SUPPORT                 0x80060001              // 不支持的功能
#define MV3D_RGBD_E_BUFOVER                 0x80060002              // 缓存已满
#define MV3D_RGBD_E_CALLORDER               0x80060003              // 函数调用顺序错误
#define MV3D_RGBD_E_PARAMETER               0x80060004              // 错误的参数
#define MV3D_RGBD_E_RESOURCE                0x80060005              // 资源申请失败
#define MV3D_RGBD_E_NODATA                  0x80060006              // 无数据
#define MV3D_RGBD_E_PRECONDITION            0x80060007              // 前置条件有误，或运行环境已发生变化
#define MV3D_RGBD_E_VERSION                 0x80060008              // 版本不匹配
#define MV3D_RGBD_E_NOENOUGH_BUF            0x80060009              // 传入的内存空间不足
#define MV3D_RGBD_E_ABNORMAL_IMAGE          0x8006000A              // 异常图像，可能是丢包导致图像不完整
#define MV3D_RGBD_E_LOAD_LIBRARY            0x8006000B              // 动态导入DLL失败
#define MV3D_RGBD_E_ALGORITHM               0x8006000C              // 算法错误
#define MV3D_RGBD_E_DEVICE_OFFLINE          0x8006000D              // 设备离线
#define MV3D_RGBD_E_ACCESS_DENIED           0x8006000E              // 设备无访问权限
#define MV3D_RGBD_E_OUTOFRANGE              0x8006000F              // 值超出范围

#define MV3D_RGBD_E_UNKNOW                  0x800600FF              // 未知的错误

/*****************************************常量定义**************************************************/
//最大的图片个数
#define MV3D_RGBD_MAX_IMAGE_COUNT           10
// 最大字符串长度
#define MV3D_RGBD_MAX_STRING_LENGTH         256
#define MV3D_RGBD_MAX_PATH_LENGTH           256
// 最大枚举数量
#define MV3D_RGBD_MAX_ENUM_COUNT            16

/*****************************************像素类型**************************************************/
#define MV3D_RGBD_PIXEL_MONO                0x01000000              
#define MV3D_RGBD_PIXEL_COLOR               0x02000000              
#define MV3D_RGBD_PIXEL_CUSTOM              0x80000000              
#define MV3D_RGBD_PIXEL_BIT_COUNT(n)        ((n) << 16)             

/*****************************************属性枚举常量**********************************************/
//设备属性key值相关定义
#define MV3D_RGBD_INT_WIDTH                 "Width"                 // 图像宽
#define MV3D_RGBD_INT_HEIGHT                "Height"                // 图像高
#define MV3D_RGBD_ENUM_WORKINGMODE          "CameraWorkingMode"     // 工作模式
#define MV3D_RGBD_ENUM_PIXELFORMAT          "PixelFormat"           // 像素格式
#define MV3D_RGBD_ENUM_IMAGEMODE            "ImageMode"             // 图像模式
#define MV3D_RGBD_FLOAT_GAIN                "Gain"                  // 增益
#define MV3D_RGBD_FLOAT_EXPOSURETIME        "ExposureTime"          // 曝光时间
#define MV3D_RGBD_FLOAT_FRAMERATE           "AcquisitionFrameRate"  // 采集帧率
#define MV3D_RGBD_ENUM_TRIGGERSELECTOR      "TriggerSelector"       // 触发选择器
#define MV3D_RGBD_ENUM_TRIGGERMODE          "TriggerMode"           // 触发模式
#define MV3D_RGBD_ENUM_TRIGGERSOURCE        "TriggerSource"         // 触发源
#define MV3D_RGBD_FLOAT_TRIGGERDELAY        "TriggerDelay"          // 触发延迟时间
#define MV3D_RGBD_INT_IMAGEALIGN            "ImageAlign"            // 深度图对齐到RGB坐标系（默认值1：对齐；0：不对齐），重启程序后恢复默认值
#define MV3D_RGBD_INT_RESOLUTIONALIGN       "ResolutionAlign"       // RGB图像分辨率与深度图对齐（默认值1：对齐；0：不对齐），重启程序后恢复默认值
#define MV3D_RGBD_BOOL_LASERENABLE          "LaserEnable"           // 投射器使能
#define Mv3D_RGBD_FLOAT_BASEDISTANCE        "BaseDistance"          // 左右目基线距
#define MV3D_RGBD_ENUM_RESOLUTION           "BinningSelector"       // 采集分辨率

/*****************************************设备文件枚举常量**********************************************/
#define MV3D_RGBD_SENSOR_CALI               "RGBDSensorCali"        // 相机传感器标定文件
#define MV3D_RGBD_HANDEYE_CALI              "RGBDHandEyeCali"       // 相机手眼标定文件

typedef int32_t                             MV3D_RGBD_STATUS;       // 返回值类型
typedef void*                               HANDLE;                 // 句柄类型
typedef int32_t                             BOOL;                   // BOOL类型

#ifndef TRUE
#define TRUE                                1                       
#endif
#ifndef FALSE
#define FALSE                               0                       
#endif
#ifndef NULL
#define NULL                                0                       
#endif

/*******************enum枚举***************************/
//设备类型：网口、USB
typedef enum Mv3dRgbdDeviceType
{
    DeviceType_Ethernet         = 1 << 0,                       
    DeviceType_USB              = 1 << 1                        
} Mv3dRgbdDeviceType;

typedef enum Mv3dRgbdIpCfgMode
{
    IpCfgMode_Static            = 1,                                // 静态IP
    IpCfgMode_DHCP              = 2,                                // 自动分配IP(DHCP)
    IpCfgMode_LLA               = 4                                 // 自动分配IP(LLA)
} Mv3dRgbdIpCfgMode;

typedef enum Mv3dRgbdUsbProtocol
{
    UsbProtocol_USB2            = 1,                                
    UsbProtocol_USB3            = 2                                 
} Mv3dRgbdUsbProtocol;

typedef enum Mv3dRgbdImageType
{
    ImageType_Undefined         = MV3D_RGBD_UNDEFINED,
    ImageType_Mono8             = (MV3D_RGBD_PIXEL_MONO  | MV3D_RGBD_PIXEL_BIT_COUNT(8)  | 0x0001),   //0x01080001
    ImageType_Depth             = (MV3D_RGBD_PIXEL_MONO  | MV3D_RGBD_PIXEL_BIT_COUNT(16) | 0x00B8),   //0x011000B8
    ImageType_YUV422            = (MV3D_RGBD_PIXEL_COLOR | MV3D_RGBD_PIXEL_BIT_COUNT(16) | 0x0032),   //0x02100032
    ImageType_YUV420SP_NV12     = (MV3D_RGBD_PIXEL_COLOR | MV3D_RGBD_PIXEL_BIT_COUNT(12) | 0x8001),   //0x020C8001
    ImageType_YUV420SP_NV21     = (MV3D_RGBD_PIXEL_COLOR | MV3D_RGBD_PIXEL_BIT_COUNT(12) | 0x8002),   //0x020C8002
    ImageType_RGB8_Planar       = (MV3D_RGBD_PIXEL_COLOR | MV3D_RGBD_PIXEL_BIT_COUNT(24) | 0x0021),   //0x02180021
    ImageType_PointCloud        = (MV3D_RGBD_PIXEL_COLOR | MV3D_RGBD_PIXEL_BIT_COUNT(96) | 0x00C0),   //0x026000C0,
    ImageType_Jpeg              = (MV3D_RGBD_PIXEL_CUSTOM| MV3D_RGBD_PIXEL_BIT_COUNT(24) | 0x0001)    //0x80180001, 自定义的图片格式
} Mv3dRgbdImageType;

enum Mv3dRgbdResolutionType
{
    Mv3dRgbdResolution_1280_720 = 0x00010001,   /**< resolution 1280x720 Bining off */
    Mv3dRgbdResolution_640_360  = 0x00020002,   /**< resolution 640x360 Bining  2*2 */
};

// 坐标系类型
typedef enum Mv3dRgbdCoordinateType
{
    CoordinateType_Depth        = 1,                                // 深度图坐标系
    CoordinateType_RGB          = 2                                 // RGB图坐标系
} Mv3dRgbdCoordinateType;

// 异常信息
typedef enum Mv3dRgbdDevException
{
    DevException_Disconnect     = 1                                 // 设备断开连接
} Mv3dRgbdDevException;

typedef enum Mv3dRgbdParamType
{
    ParamType_Bool              = 1,                                
    ParamType_Int               = 2,                                
    ParamType_Float             = 3,                                
    ParamType_Enum              = 4,                                
    ParamType_String            = 5                                 
} Mv3dRgbdParamType;

/*******************struct结构体***********************/

typedef struct _MV3D_RGBD_VERSION_INFO_
{
    uint32_t                    nMajor;                             // 主版本
    uint32_t                    nMinor;                             // 次版本
    uint32_t                    nRevision;                          // 修正版本
} MV3D_RGBD_VERSION_INFO;

//枚举相关结构体
typedef struct _MV3D_RGBD_DEVICE_NET_INFO_
{
    unsigned char               chMacAddress[8];                    // Mac地址
    Mv3dRgbdIpCfgMode           enIPCfgMode;                        // 当前IP类型
    char                        chCurrentIp[16];                    // 设备当前IP
    char                        chCurrentSubNetMask[16];            // 设备当前子网掩码BinningSelector
    char                        chDefultGateWay[16];                // 设备默认网关
    char                        chNetExport[16];                    // 网口IP地址

    uint8_t                     nReserved[16];                      // 保留字节 
} MV3D_RGBD_DEVICE_NET_INFO;

typedef struct _MV3D_RGBD_DEVICE_USB_INFO_
{
    uint32_t                    nVendorId;                          // 供应商ID号 
    uint32_t                    nProductId;                         // 产品ID号 
    Mv3dRgbdUsbProtocol         enUsbProtocol;                      // 支持的USB协议
    char                        chDeviceGUID[64];                   // 设备GUID号

    uint8_t                     nReserved[16];                      // 保留字节 
} MV3D_RGBD_DEVICE_USB_INFO;

typedef struct _MV3D_RGBD_DEVICE_INFO_
{
    char                        chManufacturerName[32];             // 设备厂商
    char                        chModelName[32];                    // 设备型号
    char                        chDeviceVersion[32];                // 设备版本
    char                        chManufacturerSpecificInfo[48];     // 设备厂商特殊信息
    char                        chSerialNumber[16];                 // 设备序列号
    char                        chUserDefinedName[16];              // 设备用户自定义名称

    Mv3dRgbdDeviceType          enDeviceType;                       // 设备类型：网口、USB
    union
    {
        MV3D_RGBD_DEVICE_NET_INFO   stNetInfo;                      // 网口设备特有
        MV3D_RGBD_DEVICE_USB_INFO   stUsbInfo;                      // USB设备特有
    } SpecialInfo;

} MV3D_RGBD_DEVICE_INFO;

typedef struct _MV3D_RGBD_IP_CONFIG_
{
    Mv3dRgbdIpCfgMode           enIPCfgMode;                        // IP配置模式
    char                        chDestIp[16];                       // 设置的目标IP,仅静态IP模式下有效
    char                        chDestNetMask[16];                  // 设置的目标子网掩码,仅静态IP模式下有效
    char                        chDestGateWay[16];                  // 设置的目标网关,仅静态IP模式下有效

    uint8_t                     nReserved[16];                      // 保留字节
} MV3D_RGBD_IP_CONFIG;

//相机图像
typedef struct _MV3D_RGBD_IMAGE_DATA_
{
    Mv3dRgbdImageType           enImageType;                        // 图像格式
    uint32_t                    nWidth;                             // 图像宽
    uint32_t                    nHeight;                            // 图像高
    uint8_t*                    pData;                              // 相机输出的图像数据
    uint32_t                    nDataLen;                           // 图像数据长度(字节)
    uint32_t                    nFrameNum;                          // 帧号，代表第几帧图像
    int64_t                     nTimeStamp;                         // 设备上报的时间戳 （设备上电从0开始，规则详见设备手册）

    uint8_t                     nReserved[16];                      // 保留字节
} MV3D_RGBD_IMAGE_DATA;

typedef struct _MV3D_RGBD_FRAME_DATA_
{
    uint32_t                    nImageCount;                                    // 图像个数,表示stImage数组的有效个数
    MV3D_RGBD_IMAGE_DATA        stImageData[MV3D_RGBD_MAX_IMAGE_COUNT];         // 图像数组，每一个代表一种类型的图像    
       
    uint8_t                     nReserved[16];                                  // 保留字节  
} MV3D_RGBD_FRAME_DATA;

/// 相机内参，3x3 matrix  
/// | fx|  0| cx|
/// |  0| fy| cy|
/// |  0|  0|  1|
typedef struct _MV3D_RGBD_CAMERA_INTRINSIC_
{
    float                       fData[3*3];                     
} MV3D_RGBD_CAMERA_INTRINSIC;

//相机畸变系数
typedef struct _MV3D_RGBD_CAMERA_DISTORTION_
{
    float                       fData[12];                          //k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
} MV3D_RGBD_CAMERA_DISTORTION;

//相机的内参和畸变系数信息
typedef struct _MV3D_RGBD_CALIB_INFO_
{
    MV3D_RGBD_CAMERA_INTRINSIC  stIntrinsic;                        // 内参：3x3 matrix
    MV3D_RGBD_CAMERA_DISTORTION stDistortion;                       // 畸变系数

    uint8_t                     nReserved[16];                      // 保留字节
} MV3D_RGBD_CALIB_INFO;

// Int类型值
typedef struct _MV3D_RGBD_INTPARAM_
{
    int64_t                     nCurValue;                          // 当前值
    int64_t                     nMax;                               // 最大值
    int64_t                     nMin;                               // 最小值
    int64_t                     nInc;                               // 增量值
} MV3D_RGBD_INTPARAM;

// Enum类型值
typedef struct _MV3D_RGBD_ENUMPARAM_
{
    uint32_t                    nCurValue;                                      // 当前值
    uint32_t                    nSupportedNum;                                  // 有效数据个数
    uint32_t                    nSupportValue[MV3D_RGBD_MAX_ENUM_COUNT];        // 支持的枚举类型
} MV3D_RGBD_ENUMPARAM;

// Float类型值
typedef struct _MV3D_RGBD_FLOATPARAM_
{
    float                       fCurValue;                          // 当前值
    float                       fMax;                               // 最大值
    float                       fMin;                               // 最小值
} MV3D_RGBD_FLOATPARAM;

// String类型值
typedef struct _MV3D_RGBD_STRINGPARAM_
{
    char                        chCurValue[MV3D_RGBD_MAX_STRING_LENGTH];        // 当前值
    uint32_t                    nMaxLength;                                     // 属性节点能设置字符的最大长度
} MV3D_RGBD_STRINGPARAM;

typedef struct _MV3D_RGBD_PARAM_
{
    Mv3dRgbdParamType           enParamType;                        //设置属性值类型
    union
    {
        BOOL                    bBoolParam;                     
        MV3D_RGBD_INTPARAM      stIntParam;                     
        MV3D_RGBD_FLOATPARAM    stFloatParam;                   
        MV3D_RGBD_ENUMPARAM     stEnumParam;                    
        MV3D_RGBD_STRINGPARAM   stStringParam;                  
    } ParamInfo;

    uint8_t                     nReserved[16];                      // 保留字节
} MV3D_RGBD_PARAM;

//异常信息
typedef struct _MV3D_RGBD_EXCEPTION_INFO_
{
    Mv3dRgbdDevException        enExceptionId;                                  // 异常ID
    char                        chExceptionDes[MV3D_RGBD_MAX_STRING_LENGTH];    // 异常描述

    uint8_t                     nReserved[4];                                   // 保留字节
} MV3D_RGBD_EXCEPTION_INFO;


//回调接口
typedef void(__stdcall* MV3D_RGBD_FrameDataCallBack)    (MV3D_RGBD_FRAME_DATA* pstFrameData, void* pUser);
typedef void(__stdcall* MV3D_RGBD_ExceptionCallBack)    (MV3D_RGBD_EXCEPTION_INFO* pstExceptInfo, void* pUser);

#endif  //  _MV3D_RGBD_DEFINE_H_
