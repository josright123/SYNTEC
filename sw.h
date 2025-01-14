//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
//
// Use of this sample source code is subject to the terms of the Microsoft
// license agreement under which you licensed this sample source code. If
// you did not accept the terms of the license agreement, you are not
// authorized to use this sample source code. For the terms of the license,
// please see the license agreement between you and Microsoft or, if applicable,
// see the LICENSE.RTF on your install media or the root of your tools installation.
// THE SAMPLE SOURCE CODE IS PROVIDED "AS IS", WITH NO WARRANTIES.
//
/*++
THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
PARTICULAR PURPOSE.

Module Name:

    sw.h

Abstract:

    The main header for an Miniport driver - Software Related.

Notes:

--*/

#ifndef _SFT_
#define _SFT_

#define DM_NDIS_MAJOR_VERSION      4  //#define DM_NDIS_MAJOR_VERSION    5 //#define PRJ_NDIS_MAJOR_VERSION   4
#define DM_NDIS_MINOR_VERSION      0  //#define DM_NDIS_MINOR_VERSION    1 //#define PRJ_NDIS_MINOR_VERSION   0

//ULONG
typedef	unsigned long	U32;
typedef	unsigned short	U16;
typedef	unsigned char	U8;
typedef	unsigned long	*PU32;
typedef	unsigned short	*PU16;
typedef	unsigned char	*PU8;

#define	DIM(a)	(sizeof(a) / sizeof(a[0]))
#define	HIGH_BYTE(n)	(((n)&0xFF00)>>8)
#define	LOW_BYTE(n)		((n)&0x00FF)
#define	HIGH_WORD(n)	(((n)&0xFFFF0000)>>16)
#define	LOW_WORD(n)		((n)&0x0000FFFF)

#define	MAKE_MASK(a)		MAKE_MASK1(a)
#define	MAKE_MASK1(a)		(1<<(a))
#define	MAKE_MASK2(a,b)		(MAKE_MASK1(a)|MAKE_MASK1(b))
#define	MAKE_MASK3(a,b,c)	(MAKE_MASK2(a,b)|MAKE_MASK1(c))
#define	MAKE_MASK4(a,b,c,d)	(MAKE_MASK2(a,b)|MAKE_MASK2(c,d))

typedef	enum {
	CID_CONNECTION_TYPE=0,
	CID_SLOT_NUMBER= 1,
	CID_BUFFER_PHYSICAL_ADDRESS= 2,
	CID_TXBUFFER_NUMBER= 3,
	CID_RXBUFFER_NUMBER= 4,
	CID_ADAPTER_NUMBER= 5,
	CID_IO_BASE_ADDRESS= 6,
	CID_IO_RANGE= 7,
	CID_IRQ_NUMBER= 8,		// irq pin or line number
	CID_IRQ_LEVEL= 9,		// to raise irql level
	CID_IRQ_GEN_TYPE= 10,	// level sensitive(pci) or latched(isa)
	CID_IRQ_SHARED= 11,		// shared or not
	CID_INTERFACE_TYPE= 12,	// isa or pci device
	CID_BUS_MASTER= 13,		// is a bus master or not
	CID_INTERMEDIATE= 14,	// is a intermediate miniport
	CID_CHECK_FOR_HANG_PERIOD= 15,	// in seconds
	CID_CHIP_STEPPING= 16,
	CID_NEED_IO_SPACE= 17,
	CID_NEED_INTERRUPT= 18,
	/* wireless settings */
	CID_WLAN_NETWORK_TYPE= 19,
	CID_DATA_OFFSET,		// add new
	CID_SIZE
}; //CID_TYPE_DEF; //DEVICE_CID_TYPE;

typedef	enum {
	SID_HW_STATUS,
	SID_OP_MODE,
	SID_INT_MASK,
	SID_INT_GEN_MASK,
	SID_PORT_BASE_ADDRESS,
	SID_PHY_NUMBER,
	SID_MEDIA_SUPPORTED,
    SID_MEDIA_IN_USE,
	SID_MEDIA_CONNECTION_STATUS,

	SID_MAXIMUM_LOOKAHEAD,
	SID_MAXIMUM_FRAME_SIZE,
    SID_MAXIMUM_TOTAL_SIZE,
    SID_BUFFER_SIZE,
    SID_MAXIMUM_SEND_PACKETS,
    SID_LINK_SPEED,


	SID_GEN_MAC_OPTIONS,
	SID_802_3_PERMANENT_ADDRESS,
	SID_802_3_CURRENT_ADDRESS,
	SID_802_3_MAXIMUM_LIST_SIZE,
	SID_802_3_MULTICAST_LIST,
	SID_GEN_CURRENT_PACKET_FILTER,
	SID_GEN_TRANSMIT_BUFFER_SPACE,
	SID_GEN_RECEIVE_BUFFER_SPACE,
	SID_GEN_TRANSMIT_BLOCK_SIZE,
	SID_GEN_RECEIVE_BLOCK_SIZE,
	SID_GEN_VENDOR_ID,
	SID_GEN_VENDOR_DESCRIPTION,
	SID_GEN_CURRENT_LOOKAHEAD,
	SID_GEN_DRIVER_VERSION,
	SID_GEN_VENDOR_DRIVER_VERSION,
	SID_GEN_PROTOCOL_OPTIONS,

	SID_SIZE
}; //DEVICE_SID_TYPE;

#ifdef	IMPL_STATISTICS
typedef	enum {
    TID_GEN_XMIT,
    TID_GEN_XMIT_OK,
    TID_GEN_XMIT_ERROR,

	TID_GEN_RCV_OK,
	TID_GEN_RCV_ERROR,
	TID_NIC_RXPS,
	TID_NIC_RXCI,
	TID_SIZE
}	TYPE_DEVICE_TID;
#endif 

//#define DM9000_IO_PORT_COUNT	(DM9000_DATA_OFFSET + 1)

//
// This structure contains all the information about a single
// adapter that this driver is controlling.
//
typedef struct _NE2000_ADAPTER {

    // This is the handle given by the wrapper for calling ndis functions.
    NDIS_HANDLE MiniportAdapterHandle;
    
	// Interrupt object.
    NDIS_MINIPORT_INTERRUPT Interrupt;

    //
    // The ethernet address currently in use.
    //
    UCHAR StationAddress[NE2000_LENGTH_OF_ADDRESS];
	U8		m_szEeprom[EEPROM_SIZE];
    
	U32		m_szConfigures[CID_SIZE];
	U32		m_szCurrentSettings[SID_SIZE];

	U8		m_szMulticastList[MAX_MULTICAST_LIST][ETH_ADDRESS_LENGTH];
	int		m_nMulticasts;

	int	m_nMaxTxPending;	
	int	m_nTxPendings;
	
	U32	uaddr;
	
	int		m_nIoMode;
	int		m_nIoMaxPad;

	U32	m_uRecentInterruptStatus;
	int		m_bSystemHang;
	
#ifdef	IMPL_STATISTICS
	U32		m_szStatistics[TID_SIZE];
#endif 
	U8		m_up;
	int		nNEQChkForHang;
	U8		nEQChkForHang;
	U16		now_mdra_rd;

	NDIS_SPIN_LOCK	m_spinAccessToken; //'m_SpinLock';  
                                           //Chariot Stress Test -20110701
	// Free-run-premptive
	int		QNDIS_State;
	U32		Check;
	UINT	RemainCount;
	PNDIS_BUFFER  pCurrBuffer;
	
        // dbg
	int 	m_QFindIndex, m_SFindIndex; // as parameter of a index
	int		m_PwrUntillSend;
	int		m_PwrUntillChkForHang;
	int     m_PwrUntillOnceSetInfo;
	NDIS_OID m_qOID[100];  // q OID list
	U16		ct_qOID[100]; // q OID per-count
	int		nqOID;        // number q OID used
	NDIS_OID m_sOID[100];
	U16		ct_sOID[100];
	int		nsOID;        // number s OID used
	// print untill Send
	//    [hhh] [hhh] [hhh] [hhh] [hhh] [hhh] | [hhh] [hhh] [hhh] 
	//      ctr   ctr   ctr   ctr   ctr   ctr |   ctr   ctr   ctr
	//    Total OIDs= nq, ns
	// print untill ChkForHang
	//    [hhh] [hhh] [hhh] [hhh] [hhh] [hhh] | [hhh] [hhh] [hhh]
	//      ctr   ctr   ctr   ctr   ctr   ctr |   ctr   ctr   ctr
	//    Total OIDs= nq, ns

	U32		txw;
	U32		txwno;

	U32		tpb;   // some thing to check (tx query pkt)
	U32		tpbno; // counter (tx buffer number)
	
	U32		rp;   // some thing to check (rx pkt)
	U32		rpno; // counter

	int		nLen;
	U32		headVal;
	U8		szbuff[DRIVER_BUFFER_SIZE];
	
} NE2000_ADAPTER, * PNE2000_ADAPTER;

#define TXNO_STAND_NO	6000 //20000 //2000 //150
#define TPBNO_STAND_NO	100000 //5000  //2000 
#define RPNO_STAND_NO	10000 //2000 

//tx
typedef	struct	_CQUEUE_GEN_HEADER
{
	struct	_CQUEUE_GEN_HEADER	*pNext;
	U32		uFlags;
	PVOID	pPacket;
	U16		nReserved;
	U16		nLength;
} CQUEUE_GEN_HEADER, *PCQUEUE_GEN_HEADER;

typedef	struct	_DATA_BLOCK
{
	CQUEUE_GEN_HEADER	Header;
	unsigned char		Buffer[DRIVER_BUFFER_SIZE];
} DATA_BLOCK, *PDATA_BLOCK;

#define	CQueueGetUserPointer(ptr)	\
	((PVOID)((U32)(ptr) + sizeof(CQUEUE_GEN_HEADER)))

//rx
typedef	struct	{
	U8		bState;
	U8		bStatus;
	U16		nLength;
} DM9_RX_DESCRIPTOR, *PDM9_RX_DESCRIPTOR;

struct MAC_ADDR_ARGS {
	UINT16 mac[3];   
	UINT32 ipAddress;
	UINT32 ipMask;   
	UINT32 ipRoute;  
};

//declaration
NDIS_STATUS
MiniportInitialize(
    OUT PNDIS_STATUS OpenErrorStatus,
    OUT PUINT SelectedMediumIndex,
    IN PNDIS_MEDIUM MediumArray,
    IN UINT MediumArraySize,
    IN NDIS_HANDLE MiniportAdapterHandle,
    IN NDIS_HANDLE ConfigurationHandle
    );
BOOLEAN	
MiniportCheckForHang(IN NDIS_HANDLE  MiniportAdapterContext);

void MiniportHalt(IN NDIS_HANDLE MiniportAdapterContext);
void MiniportInterruptHandler(IN NDIS_HANDLE MiniportAdapterContext);
void MiniportISRHandler(
    OUT PBOOLEAN InterruptRecognized,
    OUT PBOOLEAN QueueDpc,
    IN PVOID Context
    );
NDIS_STATUS
MiniportQueryInformation(
    IN NDIS_HANDLE MiniportAdapterContext,
    IN NDIS_OID Oid,
    IN PVOID InformationBuffer,
    IN ULONG InformationBufferLength,
    OUT PULONG BytesWritten,
    OUT PULONG BytesNeeded
    );
NDIS_STATUS
MiniportReset(
    OUT PBOOLEAN AddressingReset,
    IN NDIS_HANDLE MiniportAdapterContext
    );
NDIS_STATUS
MiniportSend(
    IN NDIS_HANDLE MiniportAdapterContext,
    IN PNDIS_PACKET Packet,
    IN UINT Flags
    );
NDIS_STATUS
MiniportSetInformation(
    IN NDIS_HANDLE MiniportAdapterContext,
    IN NDIS_OID Oid,
    IN PVOID InformationBuffer,
    IN ULONG InformationBufferLength,
    OUT PULONG BytesRead,
    OUT PULONG BytesNeeded
    );
NDIS_STATUS
Ne2000TransferData(
    OUT PNDIS_PACKET InPacket,
    OUT PUINT BytesTransferred,
    IN NDIS_HANDLE MiniportAdapterContext,
    IN NDIS_HANDLE MiniportReceiveContext,
    IN UINT ByteOffset,
    IN UINT BytesToTransfer
    );
    
    

void ID_Custom_Init(void);

// "ndis"
BOOLEAN SetConnectionStatus(PNE2000_ADAPTER Adapter, BOOLEAN bConnected);
void DriverReceiveIndication(
	PNE2000_ADAPTER Adapter,
	int		nCurr,
	PVOID	pVoid,
	int		nLength);

// "dm9isa"
U32	DeviceReadPort(PNE2000_ADAPTER Adapter, U32 uPort);
U32	DeviceWritePort(PNE2000_ADAPTER Adapter, U32 uPort, U32 uValue);
BOOLEAN DevicePolling(
    PNE2000_ADAPTER Adapter,
	U32		uPort,
	U32		uMask,
	U32		uExpected,
	U32		uInterval,
	U32		uRetries);
U16	DeviceWritePhy(
	PNE2000_ADAPTER Adapter, 
	U32		uOffset,
	U16		uValue);
void DeviceEnableInterrupt(PNE2000_ADAPTER Adapter);
void DeviceDisableInterrupt(PNE2000_ADAPTER Adapter);
U32 DeviceReadDataWithoutIncrement(PNE2000_ADAPTER Adapter);

// "device"
void EDeviceInitialize(PNE2000_ADAPTER Adapter);  // device.c
NDIS_STATUS DriverReset(PNE2000_ADAPTER Adapter, PBOOLEAN	pbAddressingReset); // 'RESET'
void DeviceOnSetupFilter_0 (PNE2000_ADAPTER Adapter);  // device.c
void DeviceOnSetupFilter_1(PNE2000_ADAPTER Adapter, U32 uFilter); // device.c
void DeviceStart(PNE2000_ADAPTER Adapter);
void Dm9LookupRxBuffers(PNE2000_ADAPTER Adapter);

// "temp"
U16	DeviceReadEeprom(PNE2000_ADAPTER Adapter, U32 uWordAddress);
void EDeviceLoadEeprom(PNE2000_ADAPTER Adapter);
void EDeviceCheckEeprom(PNE2000_ADAPTER Adapter);
void LoadMacFromRAM(PNE2000_ADAPTER Adapter);
U8 *DeviceMacAddress(PNE2000_ADAPTER Adapter, U8 *ptrBuffer);
NDIS_STATUS DriverQueryInformation(   // dm9isa.c
	    IN PNE2000_ADAPTER Adapter,
		IN int			n,
		IN NDIS_OID		Oid,
		IN PVOID		InfoBuffer, 
		IN ULONG		InfoBufferLength, 
		OUT PULONG		BytesWritten,
		OUT PULONG		BytesNeeded);

// "mis"
void DeviceSetDefaultSettings(PNE2000_ADAPTER Adapter);  // mis.c

U32	DeviceCalculateCRC32( // "mis"
	U8		*ptrBuffer,
	int		nLength,
	BOOL	bReverse);
	
#ifdef	IMPL_STATISTICS
  void DeviceReportStatistics(  // "mis"
	  PNE2000_ADAPTER Adapter,
	  U32		uEvent,
	  U32		uValue);
#endif	
	
//BOOLEAN QueryPacket( // (no used, only for 'send_indication', It can be a example readable for a send packet.)
//    IN PNE2000_ADAPTER Adapter, //for Adapter->uaddr
//	IN PNDIS_PACKET	pPacket,
//	IN UINT			uFlags);
//BOOLEAN QueryPacket_DM9( // (no used, this func try to save 1 memcpy.)
//    IN PNE2000_ADAPTER Adapter, //for Adapter->uaddr
//	IN PNDIS_PACKET	pPacket,
//	IN UINT			uFlags);
int QueryPacket_NDIS(
    IN PNE2000_ADAPTER Adapter, //for Adapter->uaddr
	IN PNDIS_PACKET	pPacket,
	IN UINT			uFlags);
BOOLEAN QueryPacket_SEND(
    IN PNE2000_ADAPTER Adapter);
	
void DM9000RawWritePortBufferUshort( // Alignment
    DWORD dwIoPort,
    PUSHORT pBuf,
    DWORD   cWords
    );
void DM9000RawReadPortBufferUshort( // Alignment
    DWORD dwIoPort,
    PUSHORT pBuf,
    DWORD   cWords
    );
PU8 DeviceWriteString(  // dm9isa.c
	PNE2000_ADAPTER Adapter, 
	PU8		ptrBuffer,
	int		nLength);
PU8 DeviceReadString(  // dm9isa.c
	PNE2000_ADAPTER Adapter, 
	PU8		ptrBuffer,
	int		nLength);

// "dbg"
BOOLEAN FindEverQOID(PNE2000_ADAPTER Adapter, NDIS_OID Oid);
BOOLEAN FindEverSOID(PNE2000_ADAPTER Adapter, NDIS_OID Oid);
void QSPrint(PNE2000_ADAPTER Adapter);
void SendSetFlag(PNE2000_ADAPTER Adapter);
void CheckHangSetFlag(PNE2000_ADAPTER Adapter);
void OnceSetInfoFlag(PNE2000_ADAPTER Adapter);
	
void PrintAdapterEEPROM(PNE2000_ADAPTER Adapter, char c);
void PrintCheckEEPROM(PNE2000_ADAPTER Adapter, char c);
//void PrintReadNetworkAddress(PU8 p, char c);
void PrintNdisReadNetAddrToEEPROM(PNE2000_ADAPTER Adapter, char c);
void PrintChipPAR(PNE2000_ADAPTER Adapter, char *str);
void PrintChipMDWARA(PNE2000_ADAPTER Adapter, char *str);

void PrintMoveMem(NDIS_OID Oid, ULONG BytesWritten, PU8 dat);
void PrintTxHead(int BytesPrinted, PU8 dat);

#endif // SFT

