//
//
// Three important for NDIS:
//  - _SetConnectionStatus_()
//  - _QueryPacket_()
//  - _DriverReceiveIndication_()
//
// Others, are misc
//
#include "precomp.h"


// "ndis"
//
// If set connected from disconnect state, return TRUE, 
// If set disconnected from connect state, return TRUE, 
// otherwise, FASLE
//
BOOLEAN SetConnectionStatus(PNE2000_ADAPTER Adapter, BOOLEAN bConnected)
{
	if (bConnected)
	{
		if (Adapter->m_szCurrentSettings[SID_MEDIA_CONNECTION_STATUS] == NdisMediaStateConnected)
			;
		else
		{
		NKDbgPrintfW(TEXT("[dm9]:[ Send connection!]\r\n"));
		Adapter->m_szCurrentSettings[SID_MEDIA_CONNECTION_STATUS] = NdisMediaStateConnected;
		NdisMIndicateStatus(Adapter->MiniportAdapterHandle, NDIS_STATUS_MEDIA_CONNECT,
                 			(PVOID) 0, 0);
	    NdisMIndicateStatusComplete(Adapter->MiniportAdapterHandle);
		return TRUE;
        }
	}
   	else
	{
		if (Adapter->m_szCurrentSettings[SID_MEDIA_CONNECTION_STATUS] == NdisMediaStateDisconnected)
			;
		else
		{
		NKDbgPrintfW(TEXT("[dm9]:[ Send disconnection!]\r\n"));
		Adapter->m_szCurrentSettings[SID_MEDIA_CONNECTION_STATUS] = NdisMediaStateDisconnected;
		NdisMIndicateStatus(Adapter->MiniportAdapterHandle, NDIS_STATUS_MEDIA_DISCONNECT,
			                (PVOID) 0, 0);
	    NdisMIndicateStatusComplete(Adapter->MiniportAdapterHandle);
		return TRUE;
        }
	}
	return FALSE;
}

/*
//#ifdef IMPL_SEND_INDICATION
//#endif
// "ndis"
//(no used, only for 'send_indication', It can be a example readable for a send packet.)
BOOLEAN QueryPacket(
    IN PNE2000_ADAPTER Adapter, //for Adapter->uaddr
	IN PNDIS_PACKET	pPacket,
	IN UINT			uFlags)
{
	//(1)
	PNDIS_BUFFER	pndisFirstBuffer;  // buffer-list
	UINT			uPhysicalBufferCount;
	UINT			uBufferCount;
	UINT			uTotalPacketLength;
	//(2)
	PNDIS_BUFFER	pndisCurrBuffer;  // buffer-list
	PVOID	ptrBuffer;
	UINT	nBuffer;
	U32		idx,check;
	//(3)
	PCQUEUE_GEN_HEADER pobj= (PCQUEUE_GEN_HEADER)Adapter->uaddr;
	PU8	pcurr = (PU8)CQueueGetUserPointer(Adapter->uaddr);
	// (1)
	NdisQueryPacket(
		pPacket, 
		&uPhysicalBufferCount, // (no used)
		&uBufferCount,  // buffer-count
		&pndisFirstBuffer,  // buffer-list
        &uTotalPacketLength);

    if (uTotalPacketLength > ETH_MAX_FRAME_SIZE) {
        return 0; //NDIS_STATUS_FAILURE;
    }
	uPhysicalBufferCount &= 0xFFFF; // (no used)
	//(2)
	for(idx=0,check=0, pndisCurrBuffer= pndisFirstBuffer;
		idx < uBufferCount;
		idx++, pndisCurrBuffer= pndisCurrBuffer->Next)
	{
		NdisQueryBuffer(
			pndisCurrBuffer,
			&ptrBuffer,
			&nBuffer);
		if(!nBuffer) continue;
		NdisMoveMemory(pcurr, ptrBuffer, nBuffer);
		pcurr += nBuffer;
		check += nBuffer;
	} // of for gathering buffer
	if(uTotalPacketLength != check) return 0; //NDIS_STATUS_FAILURE;
	pobj->pPacket = (PVOID)pPacket;
	pobj->uFlags = uFlags;
	pobj->nLength = uTotalPacketLength;	
	return 1;
}*/

// "ndis"
// param: Adpter->QNDIS_State; // State-of-the-machine
//          0, Init
//			1, Done
//          2, Query Buffer
//          3, ... (not available)
// param: Adpter->Check;  // Accumulater
// param: Adpter->RemainCount;  // NDIS_BUFFER remain count
// param: Adpter->pCurrBuffer; // pointer to NDIS_BUFFER list 
// return 0; //End-Fail, NDIS_STATUS_FAILURE;
// return 1; //Done-Success, QueryPacket__NDIS;
// return 2; //NeedContinue, State;
int QueryPacket_NDIS(
    IN PNE2000_ADAPTER Adapter, //for Adapter->uaddr
	IN PNDIS_PACKET	pPacket,
	IN UINT			uFlags)
{
	//(3)
	UINT			uPhysicalBufferCount;
	UINT			uBufferCount;
	PNDIS_BUFFER	pndisFirstBuffer;  // buffer-list
	UINT			uTotalPacketLength;
	//(2)
	PVOID			ptrBuffer;
	UINT			nBuffer;
	//(3)
	PU8	pcurr;
	PCQUEUE_GEN_HEADER pobj= (PCQUEUE_GEN_HEADER)Adapter->uaddr;

	if (Adapter->QNDIS_State==1) // IN_State== Done
		return 1;
	if (Adapter->QNDIS_State==0) // IN_State== Init
	{
		// (1)
		NdisQueryPacket(
			pPacket, 
			&uPhysicalBufferCount, // (no used)
			&uBufferCount,  // buffer-count
			&pndisFirstBuffer,  // buffer-list
			&uTotalPacketLength);

		if (uTotalPacketLength > ETH_MAX_FRAME_SIZE) {
			return 0; //NDIS_STATUS_FAILURE;
		}
	
		pobj->pPacket = (PVOID)pPacket;
		pobj->uFlags = uFlags;
		pobj->nLength = (U16)uTotalPacketLength;	

		Adapter->Check= 0; // Accumulater
		Adapter->RemainCount= uBufferCount; // NDIS_BUFFER remain count
		Adapter->pCurrBuffer= pndisFirstBuffer;
		return Adapter->QNDIS_State= 2; // CHG_Query_State //return 2;
	}

	// ****** IN_State== Query Buffer.s ******
	do {
		NdisQueryBuffer(
			Adapter->pCurrBuffer,
			&ptrBuffer,
			&nBuffer);
		Adapter->RemainCount--;
		Adapter->pCurrBuffer= Adapter->pCurrBuffer->Next;
	} while ( (!nBuffer) && Adapter->RemainCount);

	if (nBuffer) {
		pcurr = (PU8)CQueueGetUserPointer(Adapter->uaddr) + Adapter->Check;
		NdisMoveMemory(pcurr, ptrBuffer, nBuffer);
		Adapter->Check += nBuffer;
		  
		Adapter->tpb++;
	}

	if (Adapter->RemainCount) return 2; //NeedContinue
	// ****** IN_State== Query Buffer.e ******

	Adapter->QNDIS_State= 1; // CHG Done_State

	if (Adapter->Check) Adapter->tpbno++;
	    
	if (Adapter->Check==pobj->nLength)
		return 1;
	return 0;  //NDIS_STATUS_FAILURE;
}

#define MONITOR_TXPOINTER_CYCLE_NUM    5	// log times used in dbg mode (log for recycle happen)
typedef U16 uint16_t;
typedef U32 uint32_t;
uint16_t gkeep_mdra_wt;

uint16_t calc_mdra_to(PNE2000_ADAPTER Adapter, uint16_t mdwa, int nLength) {
	uint16_t calc = mdwa;
	switch (Adapter->m_nIoMode)
	{
		case BYTE_MODE:
			break;
			
		case WORD_MODE:
			calc = (nLength + 1) / 2 * 2;
			//calc = (nLength + 1) >> 1;
			//calc =  calc << 1;
			break;

		case DWORD_MODE:
			calc = (nLength + 3) / 4 * 4;
			//calc = (nLength + 3) >> 2;
			//calc =  calc << 2;
			break;
		
		default:
			break;
	} // of switch
	/* boundary check */
	calc &= 0x3ff;
	return calc;
}

// "ndis"
// return 0; //NDIS_STATUS_FAILURE;
// return 1; //NDIS_STATUS_SUCCESS;
BOOLEAN QueryPacket_SEND(
    IN PNE2000_ADAPTER Adapter)
{
uint16_t check_mdra_wt;
uint16_t new_mdwa_wt;
uint16_t tx_out_ptr;
int n_esc = 0;

	PCQUEUE_GEN_HEADER pobj= (PCQUEUE_GEN_HEADER)Adapter->uaddr;
	if (!Adapter->Check) return 0;

	  Adapter->tpbno++;
	
tx_out_ptr = DeviceReadPort(Adapter, 0x22);
tx_out_ptr |= DeviceReadPort(Adapter, 0x23) << 8;

gkeep_mdra_wt = DeviceReadPort(Adapter, 0xfa);
gkeep_mdra_wt |= DeviceReadPort(Adapter, 0xfb) << 8;
check_mdra_wt = calc_mdra_to(Adapter, gkeep_mdra_wt, pobj->nLength);

do {
	  DeviceWriteString(Adapter, (PU8)CQueueGetUserPointer(pobj), pobj->nLength);

  new_mdwa_wt = DeviceReadPort(Adapter, 0xfa);
  new_mdwa_wt |= DeviceReadPort(Adapter, 0xfb) << 8;
  n_esc++;
  if (check_mdra_wt != new_mdwa_wt) {
	DeviceWritePort(Adapter, 0xfa, LOW_BYTE(check_mdra_wt));
	DeviceWritePort(Adapter, 0xfb, HIGH_BYTE(check_mdra_wt));
	NKDbgPrintfW(TEXT("WorkAround [dbg_IOW_esc %d.%d] mdwa.dst.to %02x%02x e %02x%02x (ntx %d) NET.tx.out_ptr %02x%02x\r\n"), // dif %02x%02x
					MONITOR_TXPOINTER_CYCLE_NUM, n_esc,
					check_mdra_wt >> 8, check_mdra_wt & 0xff,
					new_mdwa_wt >> 8, new_mdwa_wt & 0xff,
					//diff >> 8, diff & 0xff,
					n_esc, //fifoTurn_nRx,
					tx_out_ptr >> 8, tx_out_ptr & 0xff);
  }
} while ((n_esc < 5) && (check_mdra_wt != new_mdwa_wt));
	
	  Adapter->m_nTxPendings++;
	  DeviceWritePort(Adapter, DM9_TXLENH,HIGH_BYTE(pobj->nLength));
	  DeviceWritePort(Adapter, DM9_TXLENL,LOW_BYTE(pobj->nLength));
	  // TXCR<0>, issue TX request
	  DeviceWritePort(Adapter, DM9_TXCR, MAKE_MASK(0));
	  REPORT(Adapter, TID_GEN_XMIT, 1);

	  if (Adapter->tpbno==TPBNO_STAND_NO)  // 2000, 15
	  {
		NKDbgPrintfW(TEXT("[QryPkt_NDIS] %d for %d\r\n"), Adapter->tpb, Adapter->tpbno);  //ThreeCntrPrint
		Adapter->tpb= Adapter->tpbno= 0;
	  }
	  
	return 1;
}


// "ndis"
void DriverReceiveIndication(
	PNE2000_ADAPTER Adapter,
	int		nCurr,
	PVOID	pVoid,
	int		nLength)
{
	NdisMEthIndicateReceive(
		Adapter->MiniportAdapterHandle,
		(PNDIS_HANDLE)nCurr, //= 0
		(char*)pVoid,
		ETH_HEADER_SIZE, // =14
		((char*)pVoid + ETH_HEADER_SIZE),
		nLength - ETH_HEADER_SIZE,
		nLength - ETH_HEADER_SIZE);
	
	NdisMEthIndicateReceiveComplete(Adapter->MiniportAdapterHandle);
	return;
}



// "settings"
void DeviceSetDefaultSettings(PNE2000_ADAPTER Adapter)
{
	Adapter->m_szConfigures[CID_CHIP_STEPPING] = 0;

	Adapter->m_szConfigures[CID_INTERMEDIATE] = 0;
	Adapter->m_szConfigures[CID_NEED_IO_SPACE] = 1;
	Adapter->m_szConfigures[CID_NEED_INTERRUPT] = 1;
	
	Adapter->m_szCurrentSettings[SID_PHY_NUMBER] = MII_INTERNAL_PHY_ADDR;
	Adapter->m_szCurrentSettings[SID_HW_STATUS] = NdisHardwareStatusReady;
	Adapter->m_szCurrentSettings[SID_MEDIA_SUPPORTED] = NdisMedium802_3;
	Adapter->m_szCurrentSettings[SID_MEDIA_IN_USE] = NdisMedium802_3;
	Adapter->m_szCurrentSettings[SID_MEDIA_CONNECTION_STATUS] = 
	   NdisMediaStateDisconnected; 
	  SetConnectionStatus(Adapter, 1);
	Adapter->m_szCurrentSettings[SID_OP_MODE] = 0;
	
	Adapter->m_szCurrentSettings[SID_MAXIMUM_LOOKAHEAD] = ETH_MAX_FRAME_SIZE;
	Adapter->m_szCurrentSettings[SID_MAXIMUM_FRAME_SIZE] = ETH_MAX_FRAME_SIZE - ETH_HEADER_SIZE;
    Adapter->m_szCurrentSettings[SID_MAXIMUM_TOTAL_SIZE] = ETH_MAX_FRAME_SIZE;
    Adapter->m_szCurrentSettings[SID_BUFFER_SIZE] = DRIVER_BUFFER_SIZE;
    Adapter->m_szCurrentSettings[SID_MAXIMUM_SEND_PACKETS] = 1;
	Adapter->m_szCurrentSettings[SID_LINK_SPEED] = 100000;

	Adapter->m_szCurrentSettings[SID_GEN_MAC_OPTIONS] =
		NDIS_MAC_OPTION_TRANSFERS_NOT_PEND
		| NDIS_MAC_OPTION_COPY_LOOKAHEAD_DATA
		| NDIS_MAC_OPTION_RECEIVE_SERIALIZED
		| NDIS_MAC_OPTION_NO_LOOPBACK;
	
	Adapter->m_szCurrentSettings[SID_GEN_CURRENT_PACKET_FILTER] = 0; 
	Adapter->m_szCurrentSettings[SID_GEN_TRANSMIT_BUFFER_SPACE] = 
		Adapter->m_szConfigures[CID_TXBUFFER_NUMBER]
		* ETH_MAX_FRAME_SIZE;
	Adapter->m_szCurrentSettings[SID_GEN_RECEIVE_BUFFER_SPACE] = 
		Adapter->m_szConfigures[CID_RXBUFFER_NUMBER]
		* ETH_MAX_FRAME_SIZE;
	Adapter->m_szCurrentSettings[SID_GEN_TRANSMIT_BLOCK_SIZE] = ETH_MAX_FRAME_SIZE;
	Adapter->m_szCurrentSettings[SID_GEN_RECEIVE_BLOCK_SIZE] = ETH_MAX_FRAME_SIZE;
	Adapter->m_szCurrentSettings[SID_GEN_CURRENT_LOOKAHEAD] = ETH_MAX_FRAME_SIZE;
	Adapter->m_szCurrentSettings[SID_GEN_DRIVER_VERSION] = 
		(DM_NDIS_MAJOR_VERSION << 8) | DM_NDIS_MINOR_VERSION;
	Adapter->m_szCurrentSettings[SID_GEN_VENDOR_DRIVER_VERSION] = 0x01010000; 
	Adapter->m_szCurrentSettings[SID_GEN_PROTOCOL_OPTIONS] = 0;

}

#ifdef	IMPL_STATISTICS
/*******************************************************************************
 * Statistics
 *
 ********************************************************************************/
void DeviceReportStatistics(
	PNE2000_ADAPTER Adapter,
	U32		uEvent,
	U32		uValue)
{
	Adapter->m_szStatistics[uEvent] += uValue;
}
#endif


//----------------------------------------- tmp -------------------------------------------------

#define	HANDLE_QUERY(event,ptr,len)	\
	case event: \
		Adapter->ct_qOID[n] |= 0x8000; \
		if(InfoBufferLength < (*BytesNeeded=len)) \
		{ status = NDIS_STATUS_INVALID_LENGTH; break; }	\
	panswer = (void*)(ptr); *BytesWritten = len; break;
	
NDIS_STATUS DriverQueryInformation(
		IN PNE2000_ADAPTER Adapter,
		IN int			n,   // used in the MACRO 'HANDLE_QUERY'
		IN NDIS_OID		Oid,
		IN PVOID		InfoBuffer, 
		IN ULONG		InfoBufferLength, 
		OUT PULONG		BytesWritten,
		OUT PULONG		BytesNeeded)
{
	NDIS_STATUS	status = NDIS_STATUS_SUCCESS;
	PVOID	panswer;
	U32 xmit_error_else;
	
	U32 vendor_drv= 0x01010000; //m_szCurrentSettings[SID_GEN_VENDOR_DRIVER_VERSION]= ; 
	U32 max_frame= ETH_MAX_FRAME_SIZE; //Adapter->m_szCurrentSettings[SID_MAXIMUM_LOOKAHEAD]= ;
	U32 mac_option= NDIS_MAC_OPTION_TRANSFERS_NOT_PEND //Adapter->m_szCurrentSettings[SID_GEN_MAC_OPTIONS]=
		| NDIS_MAC_OPTION_COPY_LOOKAHEAD_DATA
		| NDIS_MAC_OPTION_RECEIVE_SERIALIZED
		| NDIS_MAC_OPTION_NO_LOOPBACK;
		
	U32 multi_list= DM9_MULTICAST_LIST;
	U32 max= ETH_MAX_FRAME_SIZE - ETH_HEADER_SIZE;
	U32 link_speed= 100000; //Adapter->m_szCurrentSettings[SID_LINK_SPEED] 
		
	 xmit_error_else= Adapter->m_szStatistics[TID_GEN_XMIT] - Adapter->m_szStatistics[TID_GEN_XMIT_OK];

	switch (Oid) {
	    //#define OID_GEN_SUPPORTED_LIST 0x00010101  // No need
		HANDLE_QUERY( OID_GEN_VENDOR_DRIVER_VERSION,
 			&vendor_drv,sizeof(U32));
		HANDLE_QUERY( OID_GEN_MAXIMUM_LOOKAHEAD,
			&max_frame,sizeof(U32));

		HANDLE_QUERY( OID_GEN_MAC_OPTIONS,  // (¦h¦¸)
 			&mac_option,sizeof(U32));

		HANDLE_QUERY( OID_802_3_MAXIMUM_LIST_SIZE,
 			&multi_list,sizeof(U32));
		
		//HANDLE_QUERY( OID_802_3_CURRENT_ADDRESS,
		//	DeviceMacAddress(&szbuffer[0]),ETH_ADDRESS_LENGTH);
		case OID_802_3_CURRENT_ADDRESS: 
			QPrint(TEXT("\r\n"));
			QPrint(TEXT("  < Process(Recheck):OID_802_3_CURRENT_ADDRESS > "));
			return NDIS_STATUS_SUCCESS;
			//break;
#if 0
//reference_v2082			
  HANDLE_QUERY( OID_802_3_PERMANENT_ADDRESS,
    m_pLower->DeviceMacAddress(&szbuffer[0]),ETH_ADDRESS_LENGTH);			
#endif			
        case OID_802_3_PERMANENT_ADDRESS: 
			QPrint(TEXT("\r\n"));
			QPrint(TEXT("  < Process(Recheck):OID_802_3_PERMANENT_ADDRESS > "));
			return NDIS_STATUS_SUCCESS;

		HANDLE_QUERY( OID_GEN_MAXIMUM_FRAME_SIZE,
 			&max,sizeof(U32));
 			
		HANDLE_QUERY( OID_GEN_LINK_SPEED,
 			&link_speed,sizeof(U32));
 			
		//HANDLE_QUERY( OID_GEN_MEDIA_CONNECT_STATUS,
		//	&m_pLower->m_szCurrentSettings[SID_MEDIA_CONNECTION_STATUS],sizeof(U32));
		case OID_GEN_MEDIA_CONNECT_STATUS: 
			QPrint(TEXT("\r\n"));
			QPrint(TEXT("  < Process(Recheck):OID_GEN_MEDIA_CONNECT_STATUS > \r\n"));
			return NDIS_STATUS_SUCCESS;
			//break;
			
		HANDLE_QUERY( OID_GEN_XMIT_OK,
			&Adapter->m_szStatistics[TID_GEN_XMIT_OK],sizeof(U32));   //TX, by, REPORT(Adapter, TID_GEN_XMIT_OK, 1);
		HANDLE_QUERY( OID_GEN_RCV_OK,
			&Adapter->m_szStatistics[TID_GEN_RCV_OK],sizeof(U32));
		HANDLE_QUERY( OID_GEN_XMIT_ERROR,
			//&Adapter->m_szStatistics[TID_GEN_XMIT_ERROR],sizeof(U32));   //TX, by, Add it's count while encountting error!
			  &xmit_error_else, sizeof(U32));   //TX, by, else, ...
		HANDLE_QUERY( OID_GEN_RCV_ERROR,
			&Adapter->m_szStatistics[TID_GEN_RCV_ERROR],sizeof(U32));
    
		default:
			status = NDIS_STATUS_INVALID_OID;
			break;
	} // of switch

	//if (status== NDIS_STATUS_SUCCESS)
	//     TO_PRINT(TEXT("  Process  %x \r\n"), Oid); //QPrint
//	else
//		 QPrint(TEXT("  NoProcess %x \r\n"), Oid);

	if(status == NDIS_STATUS_SUCCESS)
	{
		PrintMoveMem(Oid, *BytesWritten, (PU8)panswer ); //dbg
		NdisMoveMemory(InfoBuffer,panswer,*BytesWritten);
	}

	return status;
}

// -------------------- + ----------------------------

//#define	HANDLE_SET(event,len)	\
//	case event: if(InfoBufferLength < (*BytesNeeded=len)) \
//		{ status = NDIS_STATUS_INVALID_LENGTH; break; }	\
//		*BytesRead = len; HANDLE_SET_##event(); break;

//#define	HANDLE_SET_OID_GEN_CURRENT_PACKET_FILTER()	\
//	DeviceOnSetupFilter_1(Adapter, *(U32*)InfoBuffer)
		
//NDIS_STATUS DriverSetInformation(
//	IN NDIS_OID		Oid,
//	IN PVOID		InfoBuffer, 
//	IN ULONG		InfoBufferLength, 
//	OUT PULONG		BytesRead,
//	OUT PULONG		BytesNeeded)
//{
//	return NDIS_STATUS_SUCCESS;
//}

// -------------------- + ----------------------------

U32	DeviceCalculateCRC32(
	U8		*ptrBuffer,
	int		nLength,
	BOOL	bReverse)
{
    U32 	Crc, Carry;
    int		i, j;
    U8		CurByte;

    Crc = 0xffffffff;

	//NKDbgPrintfW(TEXT(" =")); 
    for (i = 0; i < nLength; i++) {

		//NKDbgPrintfW(TEXT(" %02x"), ptrBuffer[i]);
        CurByte = ptrBuffer[i];

        for (j = 0; j < 8; j++) {

            Carry = ((Crc & 0x80000000) ? 1 : 0) ^ (CurByte & 0x01);

            Crc <<= 1;

            CurByte >>= 1;

            if (Carry) {

                Crc = (Crc ^ 0x04c11db6) | Carry;

            }

        }

    }
	//NKDbgPrintfW(TEXT("\r\n"));

    for (i=0, Carry=0x0L; i < 32 ; i++) {
       Carry <<= 1;
       if (Crc & 0x1) {
          Carry |= 0x1L;
       }
       Crc >>= 1;
    }

    return bReverse?~Carry:Carry;

}
