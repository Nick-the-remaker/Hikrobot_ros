/***************************************************************************************************
*
* 版权信息：版权所有 (c) 2022, 杭州海康机器人技术有限公司, 保留所有权利

*   @file       Mv3dRgbdApi.h
*   @note       HangZhou Hikrobot Technology Co., Ltd. All Right Reserved.
*   @brief      RGBD Camera basic processing
*
*   @date       2022/01/06
*   @note       V1.0.0 
*   
*   @warning    版权所有
****************************************************************************************************/

#ifndef _MV3D_RGBD_API_H_
#define _MV3D_RGBD_API_H_

#include "Mv3dRgbdDefine.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************
 *  @fn     MV3D_RGBD_GetSDKVersion()
 *  @brief  获取SDK版本号
 *  @param  pstVersion                  [OUT]           版本信息
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 
                                 
 *  @fn     MV3D_RGBD_GetSDKVersion()
 *  @brief  Get SDK Version
 *  @param  pstVersion                  [OUT]           version info
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetSDKVersion(MV3D_RGBD_VERSION_INFO* pstVersion);

/************************************************************************
 *  @fn     MV3D_RGBD_Initialize()
 *  @brief  SDK运行环境初始化
 *  @param  
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 
                                 
 *  @fn     MV3D_RGBD_Initialize()
 *  @brief  SDK run environment initialization 
 *  @param  
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_Initialize();

/************************************************************************
 *  @fn     MV3D_RGBD_Release()
 *  @brief  SDK运行环境释放
 *  @param  
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 
                                 
 *  @fn     MV3D_RGBD_Release()
 *  @brief  SDK run environment release
 *  @param  
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_Release();

/************************************************************************
 *  @fn     MV3D_RGBD_GetDeviceNumber()
 *  @brief  获取当前环境中设备数量
 *  @param  nDeviceType                 [IN]            设备类型,见Mv3dRgbdDeviceType,可全部获取(DeviceType_Ethernet | DeviceType_USB)
 *  @param  pDeviceNumber               [OUT]           设备数量
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 
 
 *  @fn     MV3D_RGBD_GetDeviceNumber()
 *  @brief  Gets the number of devices in the current environment
 *  @param  nDeviceType                 [IN]            device type，refer to Mv3dRgbdDeviceType，get all(DeviceType_Ethernet | DeviceType_USB)
 *  @param  pDeviceNumber               [OUT]           device number
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetDeviceNumber(uint32_t nDeviceType, uint32_t* pDeviceNumber);

/************************************************************************
 *  @fn     MV3D_RGBD_GetDeviceList()
 *  @brief  获取设备列表
 *  @param  nDeviceType                 [IN]            设备类型,见Mv3dRgbdDeviceType,可全部获取(DeviceType_Ethernet | DeviceType_USB)
 *  @param  pstDeviceInfos              [IN OUT]        设备列表
 *  @param  nMaxDeviceCount             [IN]            设备列表缓存最大个数
 *  @param  pDeviceCount                [OUT]           填充列表中设备个数
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 
 
 *  @fn     MV3D_RGBD_GetDeviceList()
 *  @brief  Gets 3D cameras list
 *  @param  nDeviceType                 [IN]            device type，refer to Mv3dRgbdDeviceType，get all(DeviceType_Ethernet | DeviceType_USB)
 *  @param  pstDeviceInfos              [IN OUT]        devices list
 *  @param  nMaxDeviceCount             [IN]            Max Number of device list caches
 *  @param  pDeviceCount                [OUT]           number of devices in the fill list
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetDeviceList(uint32_t nDeviceType, MV3D_RGBD_DEVICE_INFO* pstDeviceInfos, uint32_t nMaxDeviceCount, uint32_t* pDeviceCount);

/************************************************************************
 *  @fn     MV3D_RGBD_OpenDevice()
 *  @brief  打开设备
 *  @param  handle                      [IN OUT]        相机句柄
 *  @param  pstDeviceInfo               [IN]            枚举的设备信息，默认为空，打开第一个相机
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 
 
 *  @fn     MV3D_RGBD_OpenDevice()
 *  @brief  open device
 *  @param  handle                      [IN OUT]        camera handle
 *  @param  pstDeviceInfo               [IN]            enum camera info. the default is null, open first camera
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_OpenDevice(HANDLE *handle, MV3D_RGBD_DEVICE_INFO* pstDeviceInfo = NULL);

/************************************************************************
 *  @fn     MV3D_RGBD_OpenDeviceByName()
 *  @brief  通过设备名称打开设备
 *  @param  handle                      [IN OUT]        相机句柄
 *  @param  chDeviceName                [IN]            设备名称
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 

 *  @fn     MV3D_RGBD_OpenDeviceByName()
 *  @brief  open device by device name
 *  @param  handle                      [IN OUT]        camera handle
 *  @param  chDeviceName                [IN]            device name
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_OpenDeviceByName(HANDLE *handle, const char* chDeviceName);

/************************************************************************
 *  @fn     MV3D_RGBD_OpenDeviceBySerialNumber()                                                                          
 *  @brief  通过序列号打开设备
 *  @param  handle                      [IN OUT]        相机句柄
 *  @param  chSerialNumber              [IN]            序列号
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 

 *  @fn     MV3D_RGBD_OpenDeviceBySerialNumber()
 *  @brief  open device by serial number
 *  @param  handle                      [IN OUT]        camera handle
 *  @param  chSerialNumber              [IN]            serial number
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_OpenDeviceBySerialNumber(HANDLE *handle, const char* chSerialNumber);

/************************************************************************
 *  @fn     MV3D_RGBD_OpenDeviceByIp()
 *  @brief  通过IP打开设备,仅网口设备有效
 *  @param  handle                      [IN OUT]        相机句柄
 *  @param  chIP                        [IN]            IP地址
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 

 *  @fn     MV3D_RGBD_OpenDeviceByIp()
 *  @brief  open device by ip，only network device is valid
 *  @param  handle                      [IN OUT]        camera handle
 *  @param  chIP                        [IN]            IP
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_OpenDeviceByIp(HANDLE *handle, const char* chIP);

/************************************************************************
 *  @fn     MV3D_RGBD_CloseDevice()
 *  @brief  关闭设备
 *  @param  handle                      [IN]            相机句柄
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 

 *  @fn     MV3D_RGBD_CloseDevice()
 *  @brief  close Device
 *  @param  handle                      [IN]            camera handle
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_CloseDevice(HANDLE *handle);

/************************************************************************
 *  @fn     MV3D_RGBD_GetDeviceInfo
 *  @brief  获取当前设备的详细信息
 *  @param  handle                      [IN]            相机句柄
 *  @param  pstDevInfo                  [IN][OUT]       返回给调用者有关相机设备信息结构体指针
 *  @return 成功,MV3D_RGBD_OK,失败,返回错误码

 *  @fn     MV3D_RGBD_GetDeviceInfo
 *  @brief  Get current device information
 *  @param  handle                      [IN]            camera handle
 *  @param  pstDevInfo                  [IN][OUT]       Structure pointer of device information
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetDeviceInfo(HANDLE handle, MV3D_RGBD_DEVICE_INFO* pstDevInfo);

/************************************************************************
 *  @fn     MV3D_RGBD_SetIpConfig
 *  @brief  配置IP,仅网口设备有效
 *  @param  chSerialNumber              [IN]            序列号
 *  @param  pstIPConfig                 [IN]            IP配置，静态IP，DHCP等
 *  @return 成功,MV3D_RGBD_OK,失败,返回错误码

 *  @fn     MV3D_RGBD_SetIpConfig
 *  @brief  IP configuration，only network device is valid 
 *  @param  chSerialNumber              [IN]            serial number
 *  @param  pstIPConfig                 [IN]            IP Config, Static IP，DHCP
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_SetIpConfig(const char* chSerialNumber, MV3D_RGBD_IP_CONFIG* pstIPConfig);

/***********************************************************************
 *  @fn     MV3D_RGBD_RegisterFrameCallBack()
 *  @brief  注册图像数据回调
 *  @param  handle                      [IN]            相机句柄
 *  @param  cbOutput                    [IN]            回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码

 *  @fn     MV3D_RGBD_RegisterFrameCallBack()
 *  @brief  register image data callback
 *  @param  handle                      [IN]            camera handle
 *  @param  cbOutput                    [IN]            Callback function pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
***********************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_RegisterFrameCallBack(HANDLE handle, MV3D_RGBD_FrameDataCallBack cbOutput, void* pUser);

/************************************************************************
 *  @fn     MV3D_RGBD_RegisterExceptionCallBack()
 *  @brief  注册异常消息回调
 *  @param  handle：                    [IN]            相机句柄
 *  @param  cbException                 [IN]            异常回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 见返回错误码

 *  @fn     MV3D_RGBD_RegisterExceptionCallBack()
 *  @brief  Register Exception Message CallBack
 *  @param  handle:                     [IN]            camera handle
 *  @param  cbException                 [IN]            Exception Message CallBack Function Pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Refer to error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_RegisterExceptionCallBack(HANDLE handle, MV3D_RGBD_ExceptionCallBack cbException, void* pUser);

/***********************************************************************
 *  @fn     MV3D_RGBD_Start()
 *  @brief  开始工作
 *  @param  handle                      [IN]            相机句柄
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码
 
 *  @fn     MV3D_RGBD_Start()
 *  @brief  start  working
 *  @param  handle                      [IN]            camera handle
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ***********************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_Start(HANDLE handle);

/***********************************************************************
 *  @fn     MV3D_RGBD_Stop()
 *  @brief  停止工作
 *  @param  handle                      [IN]            相机句柄
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码
 
 *  @fn     MV3D_RGBD_Stop()
 *  @brief  stop working
 *  @param  handle                      [IN]            camera handle
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ***********************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_Stop(HANDLE handle);

/************************************************************************
 *  @fn     MV3D_RGBD_FetchFrame()
 *  @brief  轮询方式获取帧数据
 *  @param  handle                      [IN]            相机句柄
 *  @param  pstFrameData                [IN][OUT]       数据指针
 *  @param  nTimeOut                    [IN]            超时时间（单位:毫秒）
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 
 
 *  @fn     MV3D_RGBD_FetchFrame()
 *  @brief  fetch frame data
 *  @param  handle                      [IN]            camera handle
 *  @param  pstFrameData                [IN]            data set pointer
 *  @param  nTimeOut                    [IN]            timevalue（Unit: ms）
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_FetchFrame(HANDLE handle, MV3D_RGBD_FRAME_DATA* pstFrameData, uint32_t nTimeOut);

/************************************************************************
 *  @fn     MV3D_RGBD_SoftTrigger();
 *  @brief  执行设备软触发
 *  @param  handle                      [IN]            相机句柄
 *  @return 成功,返回MV3D_RGBD_OK,失败,返回错误码

 *  @fn     MV3D_RGBD_SoftTrigger();
 *  @brief  execute camera soft trigger 
 *  @param  handle                      [IN]            camera handle
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_SoftTrigger(HANDLE handle);

/************************************************************************
 *  @fn     MV3D_RGBD_Execute();
 *  @brief  执行设备Command命令
 *  @param  handle                      [IN]            相机句柄
 *  @param  strKey                      [IN]            属性键值
 *  @return 成功,返回MV3D_RGBD_OK,失败,返回错误码

 *  @fn     MV3D_RGBD_Execute(void* handle, const char* strKey);
 *  @brief  execute camera command 
 *  @param  handle                      [IN]            camera handle
 *  @param  strKey                      [IN]            Key value
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_Execute(HANDLE handle, const char* strKey);

/************************************************************************
 *  @fn     MV3D_RGBD_GetCalibInfo();
 *  @brief  获取相机当前标定信息
 *  @param  handle                      [IN]            相机句柄
 *  @param  nCoordinateType             [IN]            坐标系类型，见Mv3dRgbdCoordinateType
 *  @param  pstCalibInfo                [IN][OUT]       输出标定信息
 *  @return 成功,返回MV3D_RGBD_OK,失败,返回错误码

 *  @fn     MV3D_RGBD_GetCalibInfo();
 *  @brief  Get Camera Current Calibration Info
 *  @param  handle                      [IN]            camera handle
 *  @param  nCoordinateType             [IN]            Coordinate Type，refer to Mv3dRgbdCoordinateType
 *  @param  pstCalibInfo                [IN][OUT]       Calibration Info
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetCalibInfo(HANDLE handle, uint32_t nCoordinateType, MV3D_RGBD_CALIB_INFO *pstCalibInfo);

/************************************************************************
 *  @fn     MV3D_RGBD_LocalUpgrade()
 *  @brief  设备升级
 *  @param  handle                      [IN]            相机句柄
 *  @param  pFilePathName               [IN]            文件名
 *  @return 成功,返回MV3D_RGBD_OK,失败,返回错误码

 *  @fn     MV3D_RGBD_LocalUpgrade
 *  @brief  Device Upgrade
 *  @param  handle                      [IN]            camera handle
 *  @param  pFilePathName               [IN]            File name
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_LocalUpgrade(HANDLE handle, const char* pFilePathName);

/************************************************************************
 *  @fn     MV3D_RGBD_GetUpgradeProcess()
 *  @brief  获取升级进度
 *  @param  handle                      [IN]            相机句柄
 *  @param  pProcess                    [OUT]           进度接收地址
 *  @return 成功,返回MV3D_RGBD_OK,失败,返回错误码

 *  @fn     MV3D_RGBD_GetUpgradeProcess
 *  @brief  Get Upgrade Progress
 *  @param  handle                      [IN]            camera handle
 *  @param  pProcess                    [OUT]           Progress receiving address
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetUpgradeProcess(HANDLE handle, uint32_t* pProcess);

/************************************************************************
 *  @fn     MV3D_RGBD_GetParam();
 *  @brief  获取相机参数值
 *  @param  handle                      [IN]            相机句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  pstParam                    [IN]            返回的相机参数结构体指针
 *  @return 成功,返回MV3D_RGBD_OK,失败,返回错误码

 *  @fn     MV3D_RGBD_GetParam();
 *  @brief  Get Camera param value
 *  @param  handle                      [IN]            camera handle
 *  @param  strKey                      [IN]            Key value
 *  @param  pstParam                    [IN]            Structure pointer of camera param
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetParam(HANDLE handle, const char* strKey, MV3D_RGBD_PARAM* pstParam);

/************************************************************************
 *  @fn     MV3D_RGBD_SetParam();
 *  @brief  设置相机参数值
 *  @param  handle                      [IN]            相机句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  pstParam                    [IN]            输入的相机参数结构体指针
 *  @return 成功,返回MV3D_RGBD_OK,失败,返回错误码

 *  @fn     MV3D_RGBD_SetParam();
 *  @brief  Set Camera param value
 *  @param  handle                      [IN]            camera handle
 *  @param  strKey                      [IN]            Key value
 *  @param  pstParam                    [IN]            Structure pointer of camera param
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_SetParam(HANDLE handle, const char* strKey, MV3D_RGBD_PARAM* pstParam);

/************************************************************************
 *  @fn     MV3D_RGBD_ExportAllParam()
 *  @brief  导出相机参数
 *  @param  handle                      [IN]            相机句柄
 *  @param  pOutFileName                [IN]            导出文件名称
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 
 
 *  @fn     MV3D_RGBD_ExportAllParam
 *  @brief  export camera param
 *  @param  handle                      [IN]            camera handle
 *  @param  pOutFileName                [IN]            Export file name
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_ExportAllParam(HANDLE handle, const char* pOutFileName);

/************************************************************************
 *  @fn     MV3D_RGBD_ImportAllParam()
 *  @brief  导入相机参数
 *  @param  handle                      [IN]            相机句柄
 *  @param  pInFileName                 [IN]            导入文件名称
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 
 
 *  @fn     MV3D_RGBD_ImportAllParam
 *  @brief  Import camera param
 *  @param  handle                      [IN]            camera handle
 *  @param  pInFileName                 [IN]            Import file name
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_ImportAllParam(HANDLE handle, const char* pInFileName);

#ifdef __cplusplus
}
#endif

#endif // _MV3D_RGBD_API_H_