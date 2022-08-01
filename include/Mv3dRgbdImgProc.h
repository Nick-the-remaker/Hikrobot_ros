/***************************************************************************************************
*
* 版权信息：版权所有 (c) 2022, 杭州海康机器人技术有限公司, 保留所有权利

*   @file       Mv3dRgbdImgProc.h
*   @note       HangZhou Hikrobot Technology Co., Ltd. All Right Reserved.
*   @brief      RGBD camera image processing
*
*   @date       2022/01/06
*   @note       V1.0.0
*   
*   @warning    版权所有
****************************************************************************************************/

#ifndef _MV3D_RGBD_IMG_PROC_H_
#define _MV3D_RGBD_IMG_PROC_H_

#include "Mv3dRgbdDefine.h"

/************************************************************************
 *  @fn     MV3D_RGBD_MapDepthToPointCloud()
 *  @brief  RGBD相机深度图像转换点云图像  
 *  @param  handle                  [IN]          句柄地址
 *  @param  pstDepthImage           [IN]          输入深度图数据
 *  @param  pstPointCloudImage      [OUT]         输出点云图数据
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 

 *  @fn     MV3D_RGBD_MapDepthToPointCloud()
 *  @brief  depth image convert to pointcloud image
 *  @param  handle                  [IN]          handle address
 *  @param  pstDepthImage           [IN]          In Depth  data
 *  @param  pstPointCloudImage      [OUT]         Out Point Cloud data
 *  @return Success, return MV3D_RGBD_OK. Failure,return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS  MV3D_RGBD_MapDepthToPointCloud(void* handle, MV3D_RGBD_IMAGE_DATA* pstDepthImage, MV3D_RGBD_IMAGE_DATA* pstPointCloudImage);

#endif