/**********************************************************************
 *
 *  ICPDAS_USBIO.h
 *
 *  Header of USB I/O Linux Library
 *
 *  v 0.0.2 2014.4.14 by WInson Chen
 *
 *    To add USBIO DI,AI,PI function name.
 *    To add USB serial device API's function name.
 *
 *  v 0.0.1 2013.4.10 by Golden Wang
 *
 *    To modify the API's function name.
 *
 *  v 0.0.0 2013.4.9 by Golden Wang
 *
 *    create
 *
 **********************************************************************/

#pragma once

/* Linux */
#include "USBIO_Device.hpp"
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>
#include <stdint.h>

typedef unsigned int BOOL;
typedef unsigned char BYTE;
typedef unsigned short WORD;
#if 1
typedef uint32_t DWORD;
#else
typedef unsigned long DWORD;
#endif

/*record Device ID & Board ID & mapping hidraw number*/
typedef struct USBIO_list
{
        char DevName[10];
        int DeviceID;
        int BoardID;
}USBIO_list;

/* USBIO Device-ID */
#define	USBIO_MINPID  0x0400
#define BiggerBoard   100
#define	USB2019		(USBIO_MINPID+19)
#define USB2026		(USBIO_MINPID+26)
#define	USB2045		(USBIO_MINPID+45)
#define	USB2051		(USBIO_MINPID+51)
#define	USB2055		(USBIO_MINPID+55)
#define	USB2060		(USBIO_MINPID+60)
#define	USB2064		(USBIO_MINPID+64)
#define USB2084		(USBIO_MINPID+84)
#define USB2055_32	(USBIO_MINPID + BiggerBoard)
#define USB2068_18	(USBIO_MINPID + BiggerBoard + 1)
#define USB2045_32	(USBIO_MINPID + BiggerBoard + 2)
#define USB2051_32	(USBIO_MINPID + BiggerBoard + 3)
#define USB2060_24	(USBIO_MINPID + BiggerBoard + 4)
#define USB2064_16	(USBIO_MINPID + BiggerBoard + 5)
#define USB2069_18	(USBIO_MINPID + BiggerBoard + 6)

#define DEV_RETURN_ERR_CODE_BASE  0
#define DEVLIB_ERR_CODE_BASE      0x10000
#define IOLIB_ERR_CODE_BASE       0x10100

#define ERR_NO_ERR                0
/* USBDEV Error Codes */
#define ERR_USBDEV_INVALID_DEV           (DEVLIB_ERR_CODE_BASE + 0)
#define ERR_USBDEV_DEV_OPENED            (DEVLIB_ERR_CODE_BASE + 1)
#define ERR_USBDEV_DEVNOTEXISTS          (DEVLIB_ERR_CODE_BASE + 2)
#define ERR_USBDEV_GETDEVINFO            (DEVLIB_ERR_CODE_BASE + 3)
#define ERR_USBDEV_ERROR_PKTSIZE         (DEVLIB_ERR_CODE_BASE + 4)
#define ERR_USBDEV_ERROR_WRITEFILE       (DEVLIB_ERR_CODE_BASE + 5)
#define ERR_USBDEV_ERROR_OPENFILE        (DEVLIB_ERR_CODE_BASE + 6)  
#define ERR_USBDEV_ERROR_CreateRxThread  (DEVLIB_ERR_CODE_BASE + 7)
#define ERR_USBDEV_ERROR_RestartRxThread (DEVLIB_ERR_CODE_BASE + 8)

/* USBIO Error Codes */
#define ERR_USBIO_COMM_TIMEOUT      (IOLIB_ERR_CODE_BASE + 0)
#define ERR_USBIO_DEV_OPENED        (IOLIB_ERR_CODE_BASE + 1)
#define ERR_USBIO_DEV_NOTOPEN       (IOLIB_ERR_CODE_BASE + 2)
#define ERR_USBIO_INVALID_RESP      (IOLIB_ERR_CODE_BASE + 3)
#define ERR_USBIO_IO_NOTSUPPORT     (IOLIB_ERR_CODE_BASE + 4)
#define ERR_USBIO_PARA_ERROR        (IOLIB_ERR_CODE_BASE + 5)
#define ERR_USBIO_BULKVALUE_ERR     (IOLIB_ERR_CODE_BASE + 6)
#define ERR_USBIO_GETDEVINFO        (IOLIB_ERR_CODE_BASE + 7)

/* Channel Status */
#define CHSTA_GOOD            0
#define CHSTA_OVER            1
#define CHSTA_UNDER           2
#define CHSTA_OPEN            3
#define CHSTA_CLOSE           4
#define CHSTA_TYPENOTSUPPORT  5

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*OnBulkValueFinishEvent) (DWORD dwCount);
typedef void (*OnEmergencyPktArriveEvent) (BYTE * byData, BYTE byLen);

/* Get the information of USB I/O modules */
char *USBIO_GetLibraryVersion(void);
char USBIO_ListDevice(USBIO_list *list, char *count);
int USBIO_OpenDevice(WORD DEVICEID, BYTE BOARDID, int *DevNum);
int USBIO_CloseDevice(BYTE HIDDev);
int USBIO_RefreshDeviceInfo(BYTE HIDDev);
//int SYNCDevice(void);
int USBIO_SetCommTimeout(BYTE HIDDev, DWORD i_dwCommTimeout);
int USBIO_GetCommTimeout(BYTE HIDDev, DWORD * o_dwCommTimeout);
int USBIO_SetAutoResetWDT(BYTE HIDDev, BOOL i_bEnable);
int USBIO_GetSoftWDTTimeout(BYTE HIDDev, DWORD * o_dwSoftWDTTimeout);
int USBIO_GetSupportIOMask(BYTE HIDDev, BYTE * o_bySupportIOMask);
int USBIO_GetDeviceID(BYTE HIDDev, DWORD * o_dwDeviceID);
int USBIO_GetFwVer(BYTE HIDDev, WORD * o_wFwVer);
int USBIO_GetDeviceNickName(BYTE HIDDev, BYTE * o_byDeviceNickName);
int USBIO_GetDeviceSN(BYTE HIDDev, BYTE * o_byDeviceSN);
int USBIO_GetDITotal(BYTE HIDDev, BYTE * o_byDITotal);
int USBIO_GetDOTotal(BYTE HIDDev, BYTE * o_byDOTotal);
int USBIO_GetAITotal(BYTE HIDDev, BYTE * o_byAITotal);
int USBIO_GetAOTotal(BYTE HIDDev, BYTE * o_byAOTotal);
int USBIO_GetPITotal(BYTE HIDDev, BYTE * o_byPITotal);
int USBIO_GetPOTotal(BYTE HIDDev, BYTE * o_byPOTotal);
int USBIO_SendRecv(BYTE HIDDev, BYTE * i_bySend, WORD wData_Len, BYTE * o_byRecv);
int USBIO_ModuleName(BYTE HIDDev, char *module);

// DEVICE (Set)
int USBIO_SetUserDefinedBoardID(BYTE HIDDev, BYTE i_byBID);
int USBIO_SetDeviceNickName(BYTE HIDDev, BYTE* i_byDeviceNickName);
int USBIO_SetSoftWDTTimeout(BYTE HIDDev, DWORD i_dwSoftWDTTimeout);
int USBIO_LoadDefault(BYTE HIDDev);
int USBIO_StopBulk(BYTE HIDDev);

// CALLBACK REGISTRATION
int USBIO_RegisterEmergencyPktEventHandle(BYTE HIDDev, OnEmergencyPktArriveEvent i_evtHandle);

/* Get DO configuration and data */
int USBIO_DO_GetPowerOnEnable(BYTE HIDDev, BYTE * o_byPowerOnEnable);
int USBIO_DO_GetSafetyEnable(BYTE HIDDev, BYTE * o_bySafetyEnable);
int USBIO_DO_GetSafetyValue(BYTE HIDDev, BYTE * o_bySafetyValue);
int USBIO_DO_GetDigitalOutputInverse(BYTE HIDDev, DWORD* o_dwInverse);
int USBIO_DO_ReadValue(BYTE HIDDev, DWORD * o_byDOValue);

/* Set DO configuration and data */
int USBIO_DO_SetPowerOnEnableToChannel(BYTE HIDDev, BYTE i_byChToSet, BYTE i_byPowerOnEnable);
int USBIO_DO_SetPowerOnEnable(BYTE HIDDev, BYTE* i_byPowerOnEnables);
int USBIO_DO_SetSafetyEnable(BYTE HIDDev, BYTE* i_bySafetyEnable);
int USBIO_DO_SetSafetyValue(BYTE HIDDev, BYTE* i_bySafetyValue);
int USBIO_DO_SetDigitalOutputInverse(BYTE HIDDev, DWORD i_dwInverse);
int USBIO_DO_WriteValue(BYTE HIDDev, DWORD* i_byDOValue);
int USBIO_DO_WriteValueToChannel(BYTE HIDDev, BYTE i_byChannel, BYTE i_byValue);

/* Get DI configuration and data */
int USBIO_DI_GetDigitalFilterWidth(BYTE HIDDev, WORD* o_wFilterWidth);
int USBIO_DI_GetDigitalValueInverse(BYTE HIDDev, DWORD* o_dwInverse);
int USBIO_DI_GetCntEdgeTrigger(BYTE HIDDev, DWORD* o_dwEdgeTrig);
int USBIO_DI_ReadValue(BYTE HIDDev, DWORD* o_byDIValue);
int USBIO_DI_ReadCounterValue(BYTE HIDDev, DWORD* o_wDICntValue);

/* Set DI configuration and data */
int USBIO_DI_SetDigitalFilterWidth(BYTE HIDDev, WORD i_wFilterWidth);
int USBIO_DI_SetDigitalValueInverse(BYTE HIDDev, DWORD i_dwInverse);
int USBIO_DI_SetCntEdgeTrigger(BYTE HIDDev, DWORD i_dwEdgeTrig);
int USBIO_DI_WriteClearCounter(BYTE HIDDev, BYTE i_byChToClr);
int USBIO_DI_WriteClearCounterByMask(BYTE HIDDev, DWORD i_dwCntClrMask);

/* AO configuration and data */
int USBIO_AO_GetTotalSupportType(BYTE HIDDev, BYTE * o_byTotalSupportType);
int USBIO_AO_GetSupportTypeCode(BYTE HIDDev, BYTE* o_bySupportTypeCode);
int USBIO_AO_GetTypeCode(BYTE HIDDev, BYTE* o_byTypeCode);
int USBIO_AO_SetTypeCode(BYTE HIDDev, BYTE* i_byTypeCodes);
int USBIO_AO_SetTypeCodeToChannel(BYTE HIDDev, BYTE i_byChToSet, BYTE i_byTypeCode);
int USBIO_AO_GetChEnable(BYTE HIDDev, BYTE* o_byChEnable);
int USBIO_AO_SetChEnable(BYTE HIDDev, BYTE* i_byChEnable);
int USBIO_AO_GetResolution(BYTE HIDDev, BYTE* o_byResolution);
int USBIO_AO_ReadExpValueHex(BYTE HIDDev, DWORD* o_dwAOValue);
int USBIO_AO_ReadExpValueFloat(BYTE HIDDev, float* o_fAOValue);
int USBIO_AO_ReadCurValueHex(BYTE HIDDev, DWORD* o_dwAOValue);
int USBIO_AO_ReadCurValueFloat(BYTE HIDDev, float* o_fAOValue);

int USBIO_AO_WriteValueHexToChannel(BYTE HIDDev, BYTE i_byChToSet, DWORD i_dwAOVal);
int USBIO_AO_WriteValueHex(BYTE HIDDev, DWORD* i_dwAOValue);
int USBIO_AO_WriteValueFloatToChannel(BYTE HIDDev, BYTE i_byChToSet, float i_fAOExpValue);
int USBIO_AO_WriteValueFloat(BYTE HIDDev, float* i_fAOExpValue);
int USBIO_AO_GetPowerOnEnable(BYTE HIDDev, BYTE* o_byPowerOnEnable);
int USBIO_AO_SetPowerOnEnable(BYTE HIDDev, BYTE* i_byPowerOnEnable);
int USBIO_AO_GetPowerOnValueHex(BYTE HIDDev, DWORD* o_dwPwrOnValue);
int USBIO_AO_GetPowerOnValueFloat(BYTE HIDDev, float* o_fPwrOnValue);

int USBIO_AO_SetPowerOnValueHexToChannel(BYTE HIDDev, BYTE i_byChToSet, DWORD i_dwPwrOnValue);
int USBIO_AO_SetPowerOnValueHex(BYTE HIDDev, DWORD* i_dwPwrOnValue);
int USBIO_AO_SetPowerOnValueFloatToChannel(BYTE HIDDev, BYTE i_byChToSet, float i_fPwrOnValue);
int USBIO_AO_SetPowerOnValueFloat(BYTE HIDDev, float* i_fPwrOnValue);
int USBIO_AO_GetSafetyEnable(BYTE HIDDev, BYTE* o_bySafetyEnable);
int USBIO_AO_SetSafetyEnable(BYTE HIDDev, BYTE* i_bySafetyEnable);
int USBIO_AO_GetSafetyValueHex(BYTE HIDDev, DWORD* o_dwSafetyValue);
int USBIO_AO_GetSafetyValueFloat(BYTE HIDDev, float* o_fSafetyValue);
int USBIO_AO_SetSafetyValueHexToChannel(BYTE HIDDev, BYTE i_byChToSet, DWORD i_dwSafetyValue);
int USBIO_AO_SetSafetyValueHex(BYTE HIDDev, DWORD* i_dwSafetyValue);
int USBIO_AO_SetSafetyValueFloatToChannel(BYTE HIDDev, BYTE i_byChToSet, float i_fSafetyValue);
int USBIO_AO_SetSafetyValueFloat(BYTE HIDDev, float* i_fSafetyValue);

/* Get AI configuration and data */
int USBIO_AI_GetTotalSupportType(BYTE HIDDev, BYTE * o_byTotalSupportType);
int USBIO_AI_GetSupportTypeCode(BYTE HIDDev, BYTE* o_bySupportTypeCode);
int USBIO_AI_GetTypeCode(BYTE HIDDev, BYTE* o_byTypeCode);
int USBIO_AI_GetChCJCOffset(BYTE HIDDev, float* o_fChCJCOffset);
int USBIO_AI_GetChEnable(BYTE HIDDev, BYTE* o_byChEnable);
int USBIO_AI_GetFilterRejection(BYTE HIDDev, BYTE* o_byFilterRejection);
int USBIO_AI_GetCJCOffset(BYTE HIDDev, float* o_fCJCOffset);
int USBIO_AI_GetCJCEnable(BYTE HIDDev, BYTE* o_byCJCEnable);
int USBIO_AI_GetWireDetectEnable(BYTE HIDDev, BYTE* o_byWireDetectEnable);
int USBIO_AI_GetResolution(BYTE HIDDev, BYTE* o_byResolution);
int USBIO_AI_ReadValueHex(BYTE HIDDev, DWORD* o_dwAIValue);
int USBIO_AI_ReadValueHexWithChSta(BYTE HIDDev, DWORD* o_dwAIValue, BYTE* o_byChStatus);
int USBIO_AI_ReadValueFloat(BYTE HIDDev, float* o_fAIValue);
int USBIO_AI_ReadValueFloatWithChSta(BYTE HIDDev, float* o_fAIValue, BYTE* o_byChStatus);
int USBIO_AI_ReadBulkValue(BYTE HIDDev, 
			   BYTE i_byStartCh,
			   BYTE i_byChTotal,
			   DWORD i_dwSampleWidth,
			   float i_fSampleRate,
			   DWORD i_dwBufferWidth,
			   DWORD* o_dwDataBuffer,
			   OnBulkValueFinishEvent i_CBFunc);

int USBIO_AI_ReadCJCValue(BYTE HIDDev, float* o_fCJCValue);

/* Set AI configuration */
int USBIO_AI_SetTypeCodeToChannel(BYTE HIDDev, BYTE i_byChToSet, BYTE i_byTypeCode);
int USBIO_AI_SetTypeCode(BYTE HIDDev, BYTE* i_byTypeCodes);
int USBIO_AI_SetChCJCOffsetToChannel(BYTE HIDDev, BYTE i_byChToSet, float i_fChCJCOffset);
int USBIO_AI_SetChCJCOffset(BYTE HIDDev, float* i_fChCJCOffsets);
int USBIO_AI_SetChEnable(BYTE HIDDev, BYTE* i_byChEnable);
int USBIO_AI_SetFilterRejection(BYTE HIDDev, BYTE i_byFilterRejection);
int USBIO_AI_SetCJCOffset(BYTE HIDDev, float i_fCJCOffset);
int USBIO_AI_SetCJCEnable(BYTE HIDDev, BYTE i_byCJCEnable);
int USBIO_AI_SetWireDetectEnable(BYTE HIDDev, BYTE i_byWireDetectEnable);

/* Get PI configuration and data */
int USBIO_PI_GetTotalSupportType(BYTE HIDDev, BYTE* o_byTotalSupportType);
int USBIO_PI_GetSupportTypeCode(BYTE HIDDev, BYTE* o_bySupportTypeCode);
int USBIO_PI_GetTypeCode(BYTE HIDDev, BYTE* o_byTypeCode);
int USBIO_PI_GetTriggerMode(BYTE HIDDev, BYTE* o_byTriggerMode);
int USBIO_PI_GetChIsolatedFlag(BYTE HIDDev, BYTE* o_byChIsolatedFlag);
int USBIO_PI_GetLPFilterEnable(BYTE HIDDev, BYTE* o_byLPFilterEnable);
int USBIO_PI_GetLPFilterWidth(BYTE HIDDev, WORD* o_wLPFilterWidth);
int USBIO_PI_ReadValue(BYTE HIDDev, DWORD* o_dwPIValue, BYTE* o_byChStatus);
int USBIO_PI_ReadCntValue(BYTE HIDDev, DWORD* o_dwCntValue, BYTE* o_byChStatus);
int USBIO_PI_ReadFreqValue(BYTE HIDDev, float* o_fFreqValue, BYTE* o_byChStatus);
int USBIO_PI_ReadBulkValue(BYTE HIDDev,
                               BYTE i_byStartCh,
                               BYTE i_byChTotal,
                               DWORD i_dwSampleWidth,
                               float i_fSampleRate,
                               DWORD i_dwBufferWidth,
                               DWORD* o_dwDataBuffer,
                               OnBulkValueFinishEvent i_CBFunc);

/* Set PI configuration and data */
int USBIO_PI_SetTypeCodeToChannel(BYTE HIDDev, BYTE i_byChToSet, BYTE i_byTypeCode);
int USBIO_PI_SetTypeCode(BYTE HIDDev, BYTE* i_byTypeCodes);
int USBIO_PI_ClearSingleChCount(BYTE HIDDev, BYTE i_byChToClr);
int USBIO_PI_ClearChCount(BYTE HIDDev, BYTE* i_byClrMask);
int USBIO_PI_ClearSingleChStatus(BYTE HIDDev, BYTE i_byChToClr);
int USBIO_PI_ClearChStatus(BYTE HIDDev, BYTE* i_byClrMask);
//int USBIO_PI_ClearAllChCount(BYTE HIDDev);
int USBIO_PI_SetTriggerModeToChannel(BYTE HIDDev, BYTE i_byChToSet, BYTE i_byTriggerMode);
int USBIO_PI_SetTriggerMode(BYTE HIDDev, BYTE* i_byTriggerModes);
int USBIO_PI_SetChIsolatedFlagToChannel(BYTE HIDDev, BYTE i_byChToSet, BOOL i_bChIsolatedFlag);
int USBIO_PI_SetChIsolatedFlag(BYTE HIDDev, BYTE* i_byChIsolatedFlag);
int USBIO_PI_SetLPFilterEnableToChannel(BYTE HIDDev, BYTE i_byChToSet, BOOL i_bLPFilterEnable);
int USBIO_PI_SetLPFilterEnable(BYTE HIDDev, BYTE* i_byLPFilterEnables);
int USBIO_PI_SetLPFilterWidthToChannel(BYTE HIDDev, BYTE i_byChToSet, WORD i_wLPFilterWidth);
int USBIO_PI_SetLPFilterWidth(BYTE HIDDev, WORD* i_wLPFilterWidths);

#ifdef __cplusplus
}
#endif
