/**********************************************************************
 *
 *  Global.h
 *
 *  Header of Global Variables
 *  
 *  v 0.0.1 2014.4.14 by Winson Chen
 *
 *    Add struct USBIO_DI_Info_St.
 *    Modify some variable.
 *
 *  v 0.0.0 2013.4.9 by Golden Wang
 *
 *    create
 *
 **********************************************************************/
#pragma once

#include "ICPDAS_USBIO.hpp"
#include "USBIO_Object_Layout.hpp"

#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef FAILURE
#define FAILURE -1
#endif

#ifndef FALSE
#define FALSE   0
#endif

#ifndef TRUE
#define TRUE    1
#endif

#ifndef false
#define false   0
#endif

#ifndef true
#define true    1
#endif

#define ICPDAS_VENDOR 0x1b5c
#define USBIO_LIBRARY_VERSION "0.0.12"

#define USBIO_MAX_SUPPORT_TYPE 50
#define USBIO_DO_MAX_CHANNEL 32
#define USBIO_DI_MAX_CHANNEL 32
#define USBIO_AI_MAX_CHANNEL 16
#define USBIO_AO_MAX_CHANNEL 16
#define USBIO_PI_MAX_CHANNEL 16
#define USBIO_NICKNAME_LENGTH 32
#define USBIO_SN_LENGTH 32

#define USBIO_MAX_PACKET_LENGTH 1024
#define USBIO_HEADER_LENGTH 10
#define HID_MAX_BYTE       65

#define SUPPORT_DO(SupportMask) ((SupportMask & 0x1) > 0)
#define SUPPORT_DI(SupportMask) ((SupportMask & 0x2) > 0)
#define SUPPORT_AO(SupportMask) ((SupportMask & 0x4) > 0)
#define SUPPORT_AI(SupportMask) ((SupportMask & 0x8) > 0)
#define SUPPORT_PO(SupportMask) ((SupportMask & 0x10) > 0)
#define SUPPORT_PI(SupportMask) ((SupportMask & 0x20) > 0)

//#define round(x) x+0.5

#define IS_PAIR_TYPE_CODE(CompType) (CompType == TYPECODE_UP_DOWN || CompType == TYPECODE_PULSE_DIRECTION || CompType == TYPECODE_AB_PHASE)

enum
{
  TYPECODE_UP                       = 0x50,
  TYPECODE_FREQUENCY                = 0x51,
  TYPECODE_CNT_WITH_BETTERY_BACKUP  = 0x52,
  TYPECODE_ENCODER                  = 0x53,
  TYPECODE_UP_DOWN                  = 0x54,
  TYPECODE_PULSE_DIRECTION          = 0x55,
  TYPECODE_AB_PHASE                 = 0x56
};

typedef struct USBIO_PARAM {  //addmyself by golden sounds like can delete
	BYTE PortOpen;
	BYTE DevPort;
	BYTE DevType;
} USBIO_PARAM, *PUSBIO_PARAM;

typedef struct USBIO_DO_Info_St {
	BYTE PowerOnEnable[USBIO_DO_MAX_CHANNEL];
	BYTE PowerOnValue[(USBIO_DO_MAX_CHANNEL + 7) / 8];
	BYTE SafetyEnable[(USBIO_DO_MAX_CHANNEL + 7) / 8];
	BYTE SafetyValue[(USBIO_DO_MAX_CHANNEL + 7) / 8];
	DWORD DOOutputInverse;
} USBIO_DO_Info_St;

typedef struct USBIO_DI_Info_St
{
	WORD  DigitalFilterWidth;
	DWORD DIValueInverse;
	DWORD DICounterEdgeTrig;
}USBIO_DI_Info_St;

typedef struct USBIO_AO_Info_St
{
        BYTE  TotalSupportType;
        BYTE  SupportTypeCode[USBIO_MAX_SUPPORT_TYPE];
        BYTE  TypeCode[USBIO_AO_MAX_CHANNEL];
        BYTE  ChEnable[(USBIO_AO_MAX_CHANNEL + 7) / 8];
        BYTE  Resolution[USBIO_AO_MAX_CHANNEL];
        float  AOTypeLowUpLimitTable[USBIO_MAX_SUPPORT_TYPE][2];
        float  AOChLowUpLomit[USBIO_AO_MAX_CHANNEL][2];
        BYTE  WireDetectEnable;
        BYTE  PowerOnEnable[USBIO_AO_MAX_CHANNEL];
        DWORD  PowerOnValue[USBIO_AO_MAX_CHANNEL];
        BYTE  SafetyEnable[(USBIO_AO_MAX_CHANNEL + 7) / 8];
        DWORD  SafetyValue[USBIO_AO_MAX_CHANNEL];
        BYTE  SlewRate[USBIO_AO_MAX_CHANNEL];
}USBIO_AO_Info_St;

typedef struct USBIO_AI_Info_St {
	BYTE TotalSupportType;
	BYTE SupportTypeCode[USBIO_MAX_SUPPORT_TYPE];
	float AITypeLowUpLimitTable[USBIO_MAX_SUPPORT_TYPE][2];
	float AIChLowUpLomit[USBIO_AI_MAX_CHANNEL][2];
	BYTE TypeCode[USBIO_AI_MAX_CHANNEL];
	WORD ChCJCOffset[USBIO_AI_MAX_CHANNEL];
	BYTE ChEnable[(USBIO_AI_MAX_CHANNEL + 7) / 8];
	BYTE DataFormat;
	BYTE FilterRejection;
	//WORD CJCOffset;
	short CJCOffset;
	BYTE CJCEnable;
	BYTE WireDetectEnable;
	BYTE Resolution[USBIO_AI_MAX_CHANNEL];
} USBIO_AI_Info_St;

typedef struct USBIO_PI_Info_St {
	BYTE TotalSupportType;
	BYTE SupportTypeCode[USBIO_MAX_SUPPORT_TYPE];
	BYTE TypeCode[USBIO_PI_MAX_CHANNEL];
	BYTE TriggerMode[USBIO_PI_MAX_CHANNEL];
	BYTE ChIsolatedFlag[(USBIO_PI_MAX_CHANNEL + 7) / 8];
	BYTE LPFilterEnable[(USBIO_PI_MAX_CHANNEL + 7) / 8];
	WORD LPFilterWidth[USBIO_PI_MAX_CHANNEL];
} 	USBIO_PI_Info_St;

//typedef void (*OnBulkValueFinishEvent) (DWORD dwCount);
//typedef void (*OnEmergencyPktArriveEvent) (BYTE * byData, BYTE byLen);

//int RegisterEmergencyPktEventHandle(BYTE HIDDev, OnEmergencyPktArriveEvent i_evtHandle);

typedef struct ICPDAS_USBIO_PARAM {
	int FD;
	int DevIdx;

	// For callback
	BYTE m_byNormalPkt[USBIO_MAX_PACKET_LENGTH];  //addmyself 1024 sendrecv packet
	BYTE m_byNormalPktArrive;
	WORD m_wCurrentAccessObj;

	int hLogPIPktThread;
	int hLogAIPktThread;
	pthread_t hDevMonitorThread;

	// For device monitor
	BOOL m_bStopDevMonitorThread;
	BOOL m_bAutoRstWDT;
	DWORD m_dwLastCmdTime;

	// For bulk data (Log)
	void *m_byActivePkt;
	DWORD m_dwActivePktIn;
	DWORD m_dwActivePktOut;

	OnEmergencyPktArriveEvent onEmergencyPktEventHandle;

	// Flag
	BOOL m_bDeviceOpened;
	BYTE m_bySupportIOMask;
	BOOL m_bDeviceComm;

	// System information
	void *USB_DEV;
	USBIO_DO_Info_St DO_Info;
	USBIO_DI_Info_St DI_Info;
	USBIO_AO_Info_St AO_Info;
	USBIO_AI_Info_St AI_Info;
	USBIO_PI_Info_St PI_Info;

	// Device information
	DWORD m_dwmsCommTimeout;
	DWORD m_dwmsSoftWDTTimeout;
	DWORD m_dwDeviceID;
	WORD m_wFwVer;
	BYTE m_byDeviceNickName[USBIO_NICKNAME_LENGTH];
	BYTE m_byDeviceSN[USBIO_SN_LENGTH];
	BYTE m_byBoardID;

	// IOs information
	BYTE m_byDITotal;
	BYTE m_byDOTotal;
	BYTE m_byAITotal;
	BYTE m_byAOTotal;
	BYTE m_byPITotal;
	BYTE m_byPOTotal;

} ICPDAS_USBIO_DEV, *PICPDAS_USBIO_DEV;

/*
typedef struct receivethread rxthread_t;
static rxthread_t *rx_thread[MAX_USB_DEVICES] = { 0 };
static int enable_rxthread[MAX_USB_DEVICES] = { 0 };
*/


