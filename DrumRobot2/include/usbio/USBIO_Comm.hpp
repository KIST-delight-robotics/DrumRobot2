/**********************************************************************
 *
 *  USBIO_Comm.h
 *
 *  Header of USB I/O Communication
 *
 *  v 0.0.0 2013.4.9 by Golden Wang
 *
 *    create
 *
 **********************************************************************/

#pragma pack(push)				/* push current alignment to stack */
#pragma pack(1)					/* set alignment to 1 byte boundary */

#include "ICPDAS_USBIO.hpp"
#include "USBIO_Object_Layout.hpp"

typedef struct BulkPktBuffer_St {
	BYTE Packet[USBIO_MAX_PACKET_LENGTH];
} BulkPktBuffer;

typedef struct BulkValueParameter_St {
	int index;
	//BYTE byStartCh;
	BYTE byChTotal;
	DWORD dwPeriod;
	//float fSampleRate;
	DWORD dwBufferWidth;
	DWORD *dwDataBuffer;
	DWORD dwTotalByte;
	void *objAddress;
	OnBulkValueFinishEvent CBFunc;
} BulkValueParameter;

#pragma pack(pop)				/* restore original alignment from stack */

/* This define the currently version of library 
 * The format is 1.0.0.0 => 1 00 00 00 in decimal
 * Ex: 1.0.1.13 => 1 00 01 13
 */
#define LIBRARY_VERSION       1000001

#define MIN_SOFTWARE_WDT_TIMEOUT  100
#define MAX_SOFTWARE_WDT_TIMEOUT  1800000

#define MAX_BULK_PKT_BUF_LEN  256
#define BULK_PKT_BUF_MASK     0xFF
#define BULK_PKT_BUF_EMPTY    (((USBDev->m_dwActivePktIn - USBDev->m_dwActivePktOut)&BULK_PKT_BUF_MASK) == 0)
#define BULK_PKT_BUF_FULL     (((m_dwActivePktIn - m_dwActivePktOut)&BULK_PKT_BUF_MASK) == MAX_BULK_PKT_BUF_LEN)

#define MAX_AI_SAMPLE_RATE_CAPCITY 10000.0f
#define MAX_PI_SAMPLE_RATE_CAPCITY 10000.0f

/* USB protocol for normal packet definition (Byte index) */
#define NOR_PKT_FUNCTION_CODE   0
#define NOR_PKT_RESERVED_BYTE1  1
#define NOR_PKT_PACKET_LENGTH   2
#define NOR_PKT_OBJECT_INDEX    4
#define NOR_PKT_DATA_OFFSET     6
#define NOR_PKT_DATA_REGION     10

#define ACT_PKT_FUNCTION_CODE   0
#define ACT_PKT_SEQUENCE_NUM    1
#define ACT_PKT_PACKET_LENGTH   2
#define ACT_PKT_DATA_REGION     4

#define USBIO_DEFAULT_COMM_TIMEOUT 100

#define CH_OUT_OF_RANGE(ChInfo) ((ChInfo & 0x1) > 0)
#define CH_BROKEN(ChInfo)       ((ChInfo & 0x2) > 0)

#define IS_ENGINEER(DataFormat) (DataFormat == 0)
#define IS_PERCENT(DataFormat)  (DataFormat == 1)
#define IS_HEX(DataFormat)      (DataFormat == 2)

#define TYPECODE_LOW  0
#define TYPECODE_HIGH 1

//#define _DEBUG_MSG
#ifdef _DEBUG_MSG
#pragma message ( "You have been enabled debug message, please make sure it is appropriated for publishing." )
#endif

#define BASE_RESOLUTION 12
// Move to Threadfun.h can work  try ifndf
/*
const DWORD pow2[13] = { 0x1000, 0x2000, 0x4000, 0x8000,
	0x10000, 0x20000, 0x40000, 0x80000,
	0x100000, 0x200000, 0x400000, 0x800000,
	0x1000000
};*/

extern void AppendProtocolHeader(BYTE byFcnCode, WORD wObjIdx, WORD wDataLen, DWORD dwDataOffset, BYTE * byPkt);
extern int PackReadPacket(WORD wObjIdx, BYTE * byData, WORD wDataLen, BYTE * byPkt);
extern int PackLargeReadPacket(WORD wObjIdx, DWORD dwOffset, BYTE * byPkt);
extern int PackWritePacket(WORD wObjIdx, BYTE * byData, WORD wDataLen, BYTE * byPkt);
extern int PackErrorPacket(WORD wObjIdx, DWORD dwErrCode, BYTE * byPkt);
extern int PackDataLogPacket(DWORD dwLogWidth, float fSamplePeriod, WORD * wObjIdx, BYTE byTotalObj, BYTE * byPkt);
extern int PackSYNCPacket(BYTE * byPkt);
