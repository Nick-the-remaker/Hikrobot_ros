#ifndef SAMPLE_COMMON_COMMON_HPP_
#define SAMPLE_COMMON_COMMON_HPP_

#include "Utils.hpp"

#include <fstream>
#include <iterator>

void setParam(void *pHandle, char *pParamName, int pValue)
{
    MV3D_RGBD_PARAM pSetValue;
    int nRet = MV3D_RGBD_OK;
    memset(&pSetValue, 0, sizeof(MV3D_RGBD_PARAM));
    nRet = MV3D_RGBD_GetParam(pHandle, pParamName, &pSetValue);
    if (nRet != MV3D_RGBD_OK)
    {
        return;
    }
    pSetValue.ParamInfo.stIntParam.nCurValue = pValue;
    ASSERT_OK(MV3D_RGBD_SetParam(pHandle, pParamName, &pSetValue));
    LOG("ParamName : %s, Changed to %Id suceess, Max Value: %Id, Min Value: %Id\r\n",
        pParamName, pSetValue.ParamInfo.stIntParam.nCurValue, pSetValue.ParamInfo.stIntParam.nMax,
        pSetValue.ParamInfo.stIntParam.nMin);
}

void setParam(void *pHandle, char *pParamName, bool pValue)
{
    MV3D_RGBD_PARAM pSetValue;
    int nRet = MV3D_RGBD_OK;
    memset(&pSetValue, 0, sizeof(MV3D_RGBD_PARAM));
    nRet = MV3D_RGBD_GetParam(pHandle, pParamName, &pSetValue);
    if (nRet != MV3D_RGBD_OK)
    {
        return;
    }
    pSetValue.ParamInfo.bBoolParam = pValue;
    ASSERT_OK(MV3D_RGBD_SetParam(pHandle, pParamName, &pSetValue));
    LOG("ParamName : %s, Changed to %d success \r\n", pParamName, pSetValue.ParamInfo.bBoolParam);
}

void setParam(void *pHandle, char *pParamName, float pValue)
{
    MV3D_RGBD_PARAM pSetValue;
    int nRet = MV3D_RGBD_OK;
    memset(&pSetValue, 0, sizeof(MV3D_RGBD_PARAM));
    nRet = MV3D_RGBD_GetParam(pHandle, pParamName, &pSetValue);
    if (nRet != MV3D_RGBD_OK)
    {
        return;
    }
    pSetValue.ParamInfo.stFloatParam.fCurValue = pValue;
    ASSERT_OK(MV3D_RGBD_SetParam(pHandle, pParamName, &pSetValue));
    LOG("ParamName : %s, Changed to %7.2f success, Max Value: %7.2f, Min Value: %5.2f \r\n",
        pParamName, pSetValue.ParamInfo.stFloatParam.fCurValue, pSetValue.ParamInfo.stFloatParam.fMax,
        pSetValue.ParamInfo.stFloatParam.fMin);
}

void setParam(void *pHandle, char *pParamName, char pValue[256])
{
    MV3D_RGBD_PARAM pSetValue;
    int nRet = MV3D_RGBD_OK;
    memset(&pSetValue, 0, sizeof(MV3D_RGBD_PARAM));
    nRet = MV3D_RGBD_GetParam(pHandle, pParamName, &pSetValue);
    if (nRet != MV3D_RGBD_OK)
    {
        return;
    }
    memcpy(pSetValue.ParamInfo.stStringParam.chCurValue, pValue, strlen(pValue));
    pSetValue.ParamInfo.stStringParam.chCurValue[strlen(pValue)] = '\0';
    ASSERT_OK(MV3D_RGBD_SetParam(pHandle, pParamName, &pSetValue));
    LOG("ParamName : %s, Changed to %s success, Current String MaxLength: %d\r\n", pParamName, pSetValue.ParamInfo.stStringParam.chCurValue, pSetValue.ParamInfo.stStringParam.nMaxLength);
}

void setParam(void *pHandle, char *pParamName, uint32_t pEnumValue)
{
    MV3D_RGBD_PARAM pSetValue;
    int nRet = MV3D_RGBD_OK;
    memset(&pSetValue, 0, sizeof(MV3D_RGBD_PARAM));
    nRet = MV3D_RGBD_GetParam(pHandle, pParamName, &pSetValue);
    if (nRet != MV3D_RGBD_OK)
    {
        return;
    }
    pSetValue.ParamInfo.stEnumParam.nCurValue = pEnumValue;
    ASSERT_OK(MV3D_RGBD_SetParam(pHandle, pParamName, &pSetValue));
    LOG("ParamName : %s, Changed to %d success, Supported Number: %d ,Support Value is : ",
        pParamName, pSetValue.ParamInfo.stEnumParam.nCurValue, pSetValue.ParamInfo.stEnumParam.nSupportedNum);
    for (int i = 0; i < pSetValue.ParamInfo.stEnumParam.nSupportedNum; i++)
    {
        LOG("[%d] ", pSetValue.ParamInfo.stEnumParam.nSupportValue[i]);
    }
    LOG("\r");
}

void print_camera_info(void *handle, MV3D_RGBD_DEVICE_INFO pDevInfo, MV3D_RGBD_CALIB_INFO *pstRgbCalibInfo, MV3D_RGBD_CALIB_INFO *pstDepthCalibInfo)
{
    MV3D_RGBD_DEVICE_INFO pstDevInfo = pDevInfo;
    MV3D_RGBD_GetDeviceInfo(handle, &pstDevInfo);

    LOG("Manufacturer: %s\r\nDevice: %s  %s\r\nSerialNmber: %s\r\n", pstDevInfo.chManufacturerName, pstDevInfo.chModelName, pstDevInfo.chDeviceVersion, pstDevInfo.chSerialNumber);

    ASSERT_OK(MV3D_RGBD_GetCalibInfo(handle, CoordinateType_RGB, pstRgbCalibInfo));
    LOG("RGB Camera Intrinsic:\r\n");
    LOG("%-15f%-15f%-15f\r\n%-15f%-15f%-15f\r\n%-15f%-15f%-15f\r\n", pstRgbCalibInfo->stIntrinsic.fData[0], pstRgbCalibInfo->stIntrinsic.fData[1], pstRgbCalibInfo->stIntrinsic.fData[2], pstRgbCalibInfo->stIntrinsic.fData[3], pstRgbCalibInfo->stIntrinsic.fData[4], pstRgbCalibInfo->stIntrinsic.fData[5], pstRgbCalibInfo->stIntrinsic.fData[6], pstRgbCalibInfo->stIntrinsic.fData[7], pstRgbCalibInfo->stIntrinsic.fData[8]);
    LOG("RGB Camera Distortion:\r\n[k1,k2,p1,p2\r\nk3,k4,k5,k6\r\ns1,s2,s3,s4]\r\n");

    LOG("%-15f%-15f%-15f%-15f\r\n%-15f%-15f%-15f%-15f\r\n%-15f%-15f%-15f%-15f\r\n", pstRgbCalibInfo->stDistortion.fData[0], pstRgbCalibInfo->stDistortion.fData[1], pstRgbCalibInfo->stDistortion.fData[2], pstRgbCalibInfo->stDistortion.fData[3], pstRgbCalibInfo->stDistortion.fData[4], pstRgbCalibInfo->stDistortion.fData[5], pstRgbCalibInfo->stDistortion.fData[6], pstRgbCalibInfo->stDistortion.fData[7], pstRgbCalibInfo->stDistortion.fData[8], pstRgbCalibInfo->stDistortion.fData[9], pstRgbCalibInfo->stDistortion.fData[10], pstRgbCalibInfo->stDistortion.fData[11]);

    LOG("------------------------------------------------------");
    ASSERT_OK(MV3D_RGBD_GetCalibInfo(handle, CoordinateType_Depth, pstDepthCalibInfo));
    LOG("Depth Camera Intrinsic:\r\n");
    LOG("%-15f%-15f%-15f\r\n%-15f%-15f%-15f\r\n%-15f%-15f%-15f\r\n", pstDepthCalibInfo->stIntrinsic.fData[0], pstDepthCalibInfo->stIntrinsic.fData[1], pstDepthCalibInfo->stIntrinsic.fData[2], pstDepthCalibInfo->stIntrinsic.fData[3], pstDepthCalibInfo->stIntrinsic.fData[4], pstDepthCalibInfo->stIntrinsic.fData[5], pstDepthCalibInfo->stIntrinsic.fData[6], pstDepthCalibInfo->stIntrinsic.fData[7], pstDepthCalibInfo->stIntrinsic.fData[8]);
    LOG("Depth Camera Distortion:\r\n[k1,k2,p1,p2\r\nk3,k4,k5,k6\r\ns1,s2,s3,s4]\r\n");

    LOG("%-15f%-15f%-15f%-15f\r\n%-15f%-15f%-15f%-15f\r\n%-15f%-15f%-15f%-15f\r\n", pstDepthCalibInfo->stDistortion.fData[0], pstDepthCalibInfo->stDistortion.fData[1], pstDepthCalibInfo->stDistortion.fData[2], pstDepthCalibInfo->stDistortion.fData[3], pstDepthCalibInfo->stDistortion.fData[4], pstDepthCalibInfo->stDistortion.fData[5], pstDepthCalibInfo->stDistortion.fData[6], pstDepthCalibInfo->stDistortion.fData[7], pstDepthCalibInfo->stDistortion.fData[8], pstDepthCalibInfo->stDistortion.fData[9], pstDepthCalibInfo->stDistortion.fData[10], pstDepthCalibInfo->stDistortion.fData[11]);
}
#endif
