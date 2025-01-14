/********************************************************************************
 * 
 * $Id: driver.c
 *
 *  //File: Main.cpp
 *
 * Copyright (c) 2011-2021 Davicom Inc.  All rights reserved.
 
    Revision History:
    Who         When        What
    --------    --------    ----------------------------------------------
    Joseph      2011-06-09  ported & created
 *
 ********************************************************************************/

#include "precomp.h"


//
// This constant is used for places where NdisAllocateMemory
// needs to be called and the HighestAcceptableAddress does
// not matter.
//
NDIS_PHYSICAL_ADDRESS HighestAcceptableMax =
    NDIS_PHYSICAL_ADDRESS_CONST(-1,-1);


/********************************************************************************************
 *
 * Trunk Functions
 *
 ********************************************************************************************/

#pragma NDIS_PAGEABLE_FUNCTION(MiniportInitialize)
extern NDIS_STATUS MiniportInitialize(
	OUT PNDIS_STATUS OpenErrorStatus,
	OUT PUINT SelectedMediaIndex, 
	IN PNDIS_MEDIUM MediaArray, 
	IN UINT MediaArraySize,
    IN NDIS_HANDLE MiniportHandle, 
	IN NDIS_HANDLE WrapperConfigHandle)
{
/*++
Routine Description:

    MiniportInitialize handler

Arguments:

    OpenErrorStatus         Not used
    SelectedMediumIndex     Place-holder for what media we are using
    MediumArray             Array of ndis media passed down to us to pick from
    MediumArraySize         Size of the array
    MiniportAdapterHandle   The handle NDIS uses to refer to us
    WrapperConfigurationContext For use by NdisOpenConfiguration

Return Value:

    NDIS_STATUS_SUCCESS unless something goes wrong

--*/
	//NIC_DRIVER_OBJECT	*pnic;
    PNE2000_ADAPTER Adapter = NULL;
    
    //
    // The handle for reading from the registry.
    //
    NDIS_HANDLE hconfig = NULL;
    
    //
    // Status of Ndis calls.
	NDIS_STATUS	Status = NDIS_STATUS_SUCCESS;
	ULONG i;
	U32 uBase;
	U32	val;
  //NDIS_STATUS	status;
	PNDIS_CONFIGURATION_PARAMETER	param;

	int iCont;
	iCont = 0;

	do
	{
		NDIS_STATUS	status;
		// Determinate media type
		for(i=0; i<MediaArraySize; i++) 
			if(MediaArray[i] == NdisMedium802_3) break;
	
		if (i == MediaArraySize) {
				DEBUGMSG(ZONE_INIT|ZONE_ERROR, (TEXT("NE2000:Initialize: No supported media\n")));
				Status = NDIS_STATUS_UNSUPPORTED_MEDIA;
				break;
		}
	
		*SelectedMediaIndex= i;
		
		//
		// Allocate memory for the adapter block now.
		//
		Status = NdisAllocateMemory( (PVOID *)&Adapter,
					   sizeof(NE2000_ADAPTER),
					   0,
					   HighestAcceptableMax
					   );

		if (Status != NDIS_STATUS_SUCCESS)
		{
			DEBUGMSG(ZONE_INIT|ZONE_ERROR, (TEXT("NE2000:Initialize: NdisAllocateMemory(NE2000_ADAPTER) failed\n")));
			break;
		}

		//
		// Clear out the adapter block, which sets all default values to FALSE,
		// or NULL.
		NdisZeroMemory (Adapter, sizeof(NE2000_ADAPTER));
		
	 // pnic->m_pLower = DeviceEntry(this,NULL)=
		memset((void*)&Adapter->m_szConfigures,0xFF,sizeof(Adapter->m_szConfigures));
		memset((void*)&Adapter->m_szCurrentSettings,0,sizeof(Adapter->m_szCurrentSettings));
#ifdef	IMPL_STATISTICS
		memset((void*)&Adapter->m_szStatistics,0,sizeof(Adapter->m_szStatistics));
#endif		
		Adapter->m_uRecentInterruptStatus = 0;
		Adapter->m_bSystemHang = 0;
	
		// Read registry configurations
		NdisOpenConfiguration(
			&Status,
			&hconfig,
			WrapperConfigHandle);

		if (Status != NDIS_STATUS_SUCCESS)
		{
			DEBUGMSG(ZONE_INIT, (TEXT("NE2000:Initialize: NdisOpenconfiguration failed 0x%x\n"), Status));
			break;
		}
		
		Adapter->MiniportAdapterHandle = MiniportHandle;
				
	  //...	   
//ok	m_pLower->DeviceSetDefaultSettings();
	//	=
		DeviceSetDefaultSettings(Adapter);
//		m_pLower->DeviceSetEepromFormat();	
//		m_pLower->DeviceRetriveConfigurations(hconfig);
	//  +Add=  
	//--------------------------------------------------------  
				// read mac addr
				{
				  //NDIS_STATUS	 Status;  // Status of Ndis calls.
					PVOID    NetAddress;
					UINT     Length;

					NdisReadNetworkAddress(&status,&NetAddress,&Length,hconfig);
					if ((Length	== ETH_ADDRESS_LENGTH) &&	(status	== NDIS_STATUS_SUCCESS))
					{
						// Save	the address that should	be used.
						NdisMoveMemory(	&Adapter->m_szEeprom[0],NetAddress,ETH_ADDRESS_LENGTH);
                                                //PrintReadNetworkAddress((PU8) NetAddress, 'p');
                                                  PrintNdisReadNetAddrToEEPROM(Adapter, 'p');
	   				}
				}
	//--------------------------------------------------------
	
	//	=
		Adapter->m_szConfigures[CID_IO_BASE_ADDRESS]=	DM9000_CAST_IO_ADDRESS; 
		Adapter->m_szConfigures[CID_DATA_OFFSET]=		DM9000_CAST_DATA_OFFSET;
		Adapter->m_szConfigures[CID_IRQ_NUMBER]=		DM9000_CAST_IRQ_NUMBER; 
			
	//	=
	  //#define	DM9000_ADDR_OFFSET		0x00
	  //#define	DM9000_DATA_OFFSET		0x04
	  //= 'DM9000_CAST_DATA_OFFSET' (in ver.h)
	  {
	  //NDIS_STATUS	 Status;  // Status of Ndis calls.
		NDIS_STRING IOStr = NDIS_STRING_CONST("IoAddress");
		NDIS_STRING IRQStr = NDIS_STRING_CONST("IrqNumber");

		NdisReadConfiguration(
			&status,
			&param,
			hconfig,
			&IOStr, //NDIS_STRING("IoAddress"), //&(pconfig->szName),
			NdisParameterHexInteger);
		if(status == NDIS_STATUS_SUCCESS)
			Adapter->m_szConfigures[CID_IO_BASE_ADDRESS] = param->ParameterData.IntegerData;
		else
			Adapter->m_szConfigures[CID_IO_BASE_ADDRESS] = DM9000_CAST_IO_ADDRESS; // default
		
		NdisReadConfiguration(
			&status,
			&param,
			hconfig,
			&IRQStr, //NDIS_STRING("IrqNumber"), //&(pconfig->szName),
			NdisParameterHexInteger);
		if(status == NDIS_STATUS_SUCCESS)
			Adapter->m_szConfigures[CID_IRQ_NUMBER] = param->ParameterData.IntegerData;
		else
			Adapter->m_szConfigures[CID_IRQ_NUMBER] = DM9000_CAST_IRQ_NUMBER; // default
	   }

//		m_pLower->EDeviceValidateConfigurations();
	//  =
		Adapter->m_szConfigures[CID_IRQ_LEVEL] = 0x0F;
		Adapter->m_szConfigures[CID_IRQ_SHARED] = TRUE;
		// set receive mode
		// <5> discard long packet
		// <4> discard CRC error packet
		// <0> rx enable
		Adapter->m_szCurrentSettings[SID_OP_MODE] = MAKE_MASK3(5,4,0);

		// Traffic-of-chariot test
		NdisAllocateSpinLock(&Adapter->m_spinAccessToken); //(&Adapter->m_SpinLock);
		
		// dbg
		Adapter->nqOID= Adapter->nsOID= 0;
		memset((void*)&Adapter->m_qOID,0,sizeof(Adapter->m_qOID));
		memset((void*)&Adapter->ct_qOID,0,sizeof(Adapter->ct_qOID));
		memset((void*)&Adapter->m_sOID,0,sizeof(Adapter->m_sOID));
		memset((void*)&Adapter->ct_sOID,0,sizeof(Adapter->ct_sOID));
		// dbg
		Adapter->txw= Adapter->txwno= 0;
		Adapter->tpb= Adapter->tpbno= Adapter->rp= Adapter->rpno= 0;

//	} while (FALSE);
	} while (iCont);
	
	if (hconfig)
	{
		NdisCloseConfiguration(hconfig);
	}
	
	if (Status != NDIS_STATUS_SUCCESS)  // failure
	{
		if (Adapter) {
			NdisFreeMemory(Adapter, sizeof(NE2000_ADAPTER), 0);
        }
		return Status;
	}
	
// [Init-Next]	
	
	if( !(Adapter->uaddr= (U32)malloc(sizeof(DATA_BLOCK))) ) 
	{
		//THROW((ERR_STRING("Insufficient memory")));
		
		if (Adapter) {
			NdisFreeMemory(Adapter, sizeof(NE2000_ADAPTER), 0);
        }
        return NDIS_STATUS_NOT_SUPPORTED; // (?) //NDIS_STATUS_SUCCESS;
	}
		
// [Init-Next]	

//	m_pLower->DeviceRegisterAdapter();
//=
    NdisMSetAttributesEx(
		Adapter->MiniportAdapterHandle,  // miniport handle 
		(NDIS_HANDLE)Adapter,		// miniport context
		3, //36, //12, //3 sec, 
		0, 
		NdisInterfaceIsa); 
		
	do {
		    //defined(DM9000)
			if((Status = NdisMRegisterIoPortRange(
				(PVOID*)&uBase,
				Adapter->MiniportAdapterHandle,
				Adapter->m_szConfigures[CID_IO_BASE_ADDRESS],
				DM9000_CAST_DATA_OFFSET + 1))
				!=NDIS_STATUS_SUCCESS){
				NKDbgPrintfW(TEXT("[dm9: Fails to map io space]\r\n"));
				return Status;
			}
				
			//if(!(uBase = (U32)MmMapIoSpace(phyAddr, 16, FALSE)))
			//	THROW((ERR_STRING("Fails to map io space")));
			Adapter->m_szCurrentSettings[SID_PORT_BASE_ADDRESS] = uBase;	

			ID_Custom_Init();
			
			val  = DeviceReadPort(Adapter, 0x2a);
			val |= DeviceReadPort(Adapter, 0x2b)<<8;
			val |= DeviceReadPort(Adapter, 0x28)<<16;			
			val |= DeviceReadPort(Adapter, 0x29)<<24;

			NKDbgPrintfW(TEXT("[dm9: Chip signature is %08X]\r\n"), val);			
			NKDbgPrintfW(TEXT("\r\n"));
		  
//--------------------------------------------------
		{
			U8 rdMAC[6];	
			int n;
			for(n=0;n<ETH_ADDRESS_LENGTH;n++)
			{
				rdMAC[n]= (U8) DeviceReadPort(Adapter, DM9_PAR0+n);
			}

		}

		if( ( ((val>>16)&0xff) != LOW_BYTE(DM9000_VID) ) ||
			( ((val>>24)&0xff) != HIGH_BYTE(DM9000_VID) ) ||
			( ((val>>0)&0xff) != LOW_BYTE(DM9000_PID) ) ||
			( ((val>>8)&0xff) != HIGH_BYTE(DM9000_PID) ) )
		{
	       	Status= NDIS_STATUS_ADAPTER_NOT_FOUND; //(TEXT("Unknown device\n")
			return Status;
	    }

		// load MAC from ARGS on RAM
		LoadMacFromRAM(Adapter);

		EDeviceInitialize(Adapter);

#if 1
		if((Status= NdisMRegisterInterrupt(
			&Adapter->Interrupt,
			Adapter->MiniportAdapterHandle,
			Adapter->m_szConfigures[CID_IRQ_NUMBER], // =DM9000_CAST_IRQ_NUMBER
			Adapter->m_szConfigures[CID_IRQ_LEVEL],	 // =0x0F, irql level
			TRUE,  // request ISR
			TRUE,  // FALSE, // shared interrupt //Adapter->m_szConfigures[CID_IRQ_SHARED], 
			NdisInterruptLatched)) != NDIS_STATUS_SUCCESS)	
		{
	    	NKDbgPrintfW(TEXT("[dm9: Error in registering interrupt]\r\n"));
			return Status;
		}
#else
		NKDbgPrintfW(TEXT("[dm9: Skip registering interrupt .........]\r\n"));
#endif		
//	} while (FALSE);
	} while (iCont);
	
    DeviceOnSetupFilter_0(Adapter);
	
	DeviceStart(Adapter);

	if (Status == NDIS_STATUS_SUCCESS)
	{
		NKDbgPrintfW(TEXT("[dm9: DM9000:Initialize succeeded]\r\n"));
	}
	
	return Status;  //NDIS_STATUS_SUCCESS;
}



/********************************************************************************************
 *
 * DriverEntry
 *
 ********************************************************************************************/
int LocalBus_setup(void)
{
	int iRet = 0;
	
 	
	
	return iRet;
	
}


NDIS_STATUS DriverEntry(
	IN PDRIVER_OBJECT	pDriverObject, 
	IN PUNICODE_STRING	pRegistryPath)
{
	NDIS_STATUS		status;
	NDIS_HANDLE		hwrapper;
    NDIS_MINIPORT_CHARACTERISTICS dmchar;

	/*RETAILMSG(1,(TEXT("[dm9]:\r\n")));
	RETAILMSG(1,(TEXT("[dm9] [CUSTOM_BOARD_STR]\r\n")));
    RETAILMSG(1,(TEXT("[dm9] NewOBJ-2011 \r\n")));
    NKDbgPrintfW(TEXT("[dm9] NewOBJ-2011 %s \r\n"), DRIVER_VERSION);
    NKDbgPrintfW(TEXT("[dm9] NewOBJ-2011 %s \r\n"), CUSTOM_BOARD_STR);*/
    
    if (LocalBus_setup()!=0)	//&*&*j1_add
    {
	    RETAILMSG(1,(TEXT("[dm9] Local bus .. configure failure... [**ERR**] \r\n")));
    }
  
    //
    // Notify the NDIS wrapper about this driver, get a NDIS wrapper handle back
    //
	NdisMInitializeWrapper(
		&hwrapper, 
		pDriverObject,
		pRegistryPath,
		NULL);

    NdisZeroMemory(&dmchar, sizeof(dmchar));
    
    dmchar.MajorNdisVersion         = DM_NDIS_MAJOR_VERSION;
    dmchar.MinorNdisVersion         = DM_NDIS_MINOR_VERSION;

	
 //   ndischar.Ndis30Chars.MajorNdisVersion = PRJ_NDIS_MAJOR_VERSION;
 //   ndischar.Ndis30Chars.MinorNdisVersion = PRJ_NDIS_MINOR_VERSION;
    
  //  ndischar.Ndis30Chars.CheckForHangHandler = MiniportCheckForHang;
  //  ndischar.Ndis30Chars.HaltHandler         = MiniportHalt;
  //  ndischar.Ndis30Chars.InitializeHandler = MiniportInitialize;
  //  ndischar.Ndis30Chars.QueryInformationHandler  = MiniportQueryInformation;

  //  ndischar.Ndis30Chars.ResetHandler      = MiniportReset;
  //  ndischar.Ndis30Chars.SetInformationHandler	  = MiniportSetInformation;
  //  ndischar.Ndis30Chars.HandleInterruptHandler   = MiniportInterruptHandler;
  //  ndischar.Ndis30Chars.ISRHandler               = MiniportISRHandler;

  //  ndischar.Ndis30Chars.SendHandler              = MiniportSend;

    dmchar.CheckForHangHandler      = MiniportCheckForHang;  // NULL; //
    dmchar.DisableInterruptHandler  = NULL;
    dmchar.EnableInterruptHandler   = NULL;
    dmchar.HaltHandler              = MiniportHalt;			// IMPL //
    dmchar.InitializeHandler        = MiniportInitialize;		// IMPL //
    dmchar.QueryInformationHandler  = MiniportQueryInformation;		// IMPL //
    //dmchar.ReconfigureHandler         = NULL;
    dmchar.ResetHandler             = MiniportReset;		// IMPL //
    #if 0
    dmchar.ReturnPacketHandler      = MPReturnPacket;
    dmchar.SendPacketsHandler       = MpSendPacketsHandler;
    #endif
    
    dmchar.SetInformationHandler    = MiniportSetInformation;		// IMPL //
    #if 0
    dmchar.AllocateCompleteHandler  = MPAllocateComplete;
    #endif
    dmchar.HandleInterruptHandler   = MiniportInterruptHandler;		// IMPL //
    dmchar.ISRHandler               = MiniportISRHandler;		// IMPL //

#if 1
	#if 0
    dmchar.CancelSendPacketsHandler = MPCancelSendPackets;
    dmchar.PnPEventNotifyHandler    = MPPnPEventNotify;
    dmchar.AdapterShutdownHandler   = MPShutdown;
    #endif
#endif
    
/*    
    ndischar.Ndis30Chars.MajorNdisVersion = PRJ_NDIS_MAJOR_VERSION;
    ndischar.Ndis30Chars.MinorNdisVersion = PRJ_NDIS_MINOR_VERSION;
	...
    ndischar.Ndis30Chars.SendHandler              = MiniportSend;
*/
    dmchar.SendHandler				= MiniportSend;		// IMPL //
    dmchar.TransferDataHandler 		= Ne2000TransferData;		// IMPL //

	if((status = NdisMRegisterMiniport(
		hwrapper,
		&dmchar,
		sizeof(NDIS_MINIPORT_CHARACTERISTICS)) != NDIS_STATUS_SUCCESS))
	{
	    //NKDbgPrintfW(TEXT("[dm9.Fail] <==== DriverEntry, Status=%x\n"), status);
		return status; // Fail
	}
	
    return status; //NDIS_STATUS_SUCCESS;

}

BOOLEAN	
MiniportCheckForHang(IN NDIS_HANDLE  MiniportAdapterContext)
{
	uint16_t rwpa_wt;
    U32 dat, val;
	U32 mdra_rd_now;
	U8 sz[6];
	BOOLEAN toup,todown;
    PNE2000_ADAPTER Adapter = (PNE2000_ADAPTER)MiniportAdapterContext;

	//CheckHangSetFlag(Adapter);
  
  	if(Adapter->m_bSystemHang) {
  		//NKDbgPrintfW(TEXT("[dm9] .MiniportCheckForHang.System Hang.To.Reset!\r\n"));
  		return TRUE;
  	}

	val  = DeviceReadPort(Adapter, 0x2a);
	val |= DeviceReadPort(Adapter, 0x2b)<<8;
	val |= DeviceReadPort(Adapter, 0x28)<<16;			
	val |= DeviceReadPort(Adapter, 0x29)<<24;
	
  rwpa_wt = (uint16_t)DeviceReadPort(Adapter, DM9051_RWPAL) |
             (uint16_t)DeviceReadPort(Adapter, DM9051_RWPAH) << 8;
             
		mdra_rd_now = DeviceReadPort(Adapter, 0xf4);
		mdra_rd_now |= DeviceReadPort(Adapter, 0xf5) << 8;

	NKDbgPrintfW(TEXT("=== [dm9] s.w.lee: Chip signature is %08X mdrd %04x] === rwpa_wt %02x%02x\r\n"), val, mdra_rd_now, 
		rwpa_wt>> 8, rwpa_wt);			

	//if (Adapter->rp == 1)
	//	diff_rx_s(Adapter); //.
	//if (Adapter->rp > 1)
	//	diff_rx_e(Adapter);

    dat= DeviceReadPort(Adapter, DM9_NSR);
    if (dat & 0x40)
	{
		if (!(Adapter->m_up++%380)) // 38->380
			NKDbgPrintfW(TEXT("[dm9 up] .MiniportCheckForHang!\r\n"));
		toup= SetConnectionStatus(Adapter, 1);
		if (toup)
		{
            //PrintChipMDWARA(Adapter, "UP.s");
			//[1] Software-Reset
			//(1) No correction effect
			//[2] DeviceWritePort(Adapter, DM9_NCR, 0x03);
			//NdisStallExecution(20);
			//NKDbgPrintfW(TEXT("[dm9] -- [Set DM9_NCR 0x03] MAC_Intl_Lpbk+SwReset: then read DM9_NCR= %02x -- \r\n"), DeviceReadPort(Adapter, DM9_NCR));
			//(2) Solved 
			EDeviceInitialize(Adapter);
		//[ DeviceOnSetupFilter_0(Adapter); ] no need
			DeviceStart(Adapter);	
            //PrintChipMDWARA(Adapter, "UP.e");
		}
	}
	else
	{
		Adapter->m_up= 0;
//j		NKDbgPrintfW(TEXT("[dm9 down] .MiniportCheckForHang!\r\n"));
		
		val = DeviceReadPort(Adapter, DM9_GPR);
//j		NKDbgPrintfW(TEXT("[dm9 down] -- ForHang Get : GPR= %02x -- (#bit-0 check, =0 PHY-ON) \r\n"), val);
	
    	todown= SetConnectionStatus(Adapter, 0);
		if (todown)
		{
            //PrintChipMDWARA(Adapter, "DOWN.p");

            //PrintChipPAR(Adapter, "DOWN.p");

			DeviceMacAddress(Adapter, &sz[0]);
			/*NKDbgPrintfW(TEXT("dm9:m_szEeprom[]= %02x %02x %02x %02x %02x %02x ..\r\n"), 
					sz[0],
					sz[1],
					sz[2],
					sz[3],
					sz[4],
					sz[5]
					);
			NKDbgPrintfW(TEXT("=\r\n"));*/

            //PrintCheckEEPROM(Adapter, 'p');
		}
	}
    	
#if 0
/*
	U16 link_state;

	// check link status
	link_state = DeviceReadPhy(Adapter, 0, 0x1);
	link_state = DeviceReadPhy(Adapter, 0, 0x1);
	RETAILMSG(debug,(TEXT("[DM9isa]Link=%d\r\n"), (link_state&0x4)));

	if (m_nOldLinkState != link_state)
	{	
		if (link_state & 0x4)
			SetConnectionStatus(1);
		else
    	SetConnectionStatus(0);
	}

	m_nOldLinkState= link_state;

	U32	cr= DeviceGetReceiveStatus();
	U32	rxps,rxci;
	
	rxps = cr >> 31;
	rxci = cr & 0x7FFFFFFF;

#ifdef	IMPL_STATISTICS
	REPORT(TID_NIC_RXPS,rxps);	
	REPORT(TID_NIC_RXCI,rxci);	
#endif 

#ifndef	IMPL_RESET
	return FALSE;
#endif

	U32	lastread = m_szLastStatistics[TID_GEN_RCV_OK];
	U32	lastsent = m_szLastStatistics[TID_GEN_XMIT_OK];

	memcpy(
		(void*)&m_szLastStatistics,
		(void*)&m_szStatistics,
		sizeof(m_szStatistics));

	// report hang if 
	// 1. receive count stalled but overflow, or
	if((m_szStatistics[TID_GEN_RCV_OK] == lastread) && cr) 
		return TRUE;
	
	// 2. tx idle while tqueue out of resource
	if(m_pUpper->DriverIsOutOfResource() &&
		(DeviceHardwareStatus() & NIC_HW_TX_IDLE) ) 
	{
		return TRUE;
	}
*/	
#endif
	return FALSE ;
}
    
extern
void MiniportHalt(IN NDIS_HANDLE MiniportAdapterContext)
{
    NKDbgPrintfW(TEXT("   [MiniportHalt] -------- [dm9] 2011 ------------------------ \r\n"));
    //NKDbgPrintfW(TEXT("     Halt=  Halt --------------------------------\r\n"));
}

void MiniportInterruptHandler(IN NDIS_HANDLE MiniportAdapterContext)
{
	U32	uValue;
    PNE2000_ADAPTER Adapter = (PNE2000_ADAPTER)MiniportAdapterContext;

	uValue= Adapter->m_uRecentInterruptStatus;

	//NKDbgPrintfW(TEXT("[dm9] -- s.w.lee .. isr, value = %d  .. MiniportInterruptHandler.. 2\r\n"), uValue );
	
	// check RX activities 
	if(uValue & 0x01) Dm9LookupRxBuffers(Adapter);

#if (DM9000_IMPL_SEND==SEND_WAIT_TXCMPLT)
	if (Adapter->m_nTxPendings)
	{
			if (!(DeviceReadPort(Adapter, 0x02) & 0x01))
			{
  //#ifdef IMPL_SEND_INDICATION
	//			NdisMSendResourcesAvailable(Adapter->MiniportAdapterHandle);
	//			NdisMSendComplete(
	//					Adapter->MiniportAdapterHandle,
	//					(PNDIS_PACKET)(((PCQUEUE_GEN_HEADER)Adapter->uaddr)->pPacket),
	//					NDIS_STATUS_SUCCESS);
	//			if (Adapter->m_nTxPendings){
	//				NKDbgPrintfW(TEXT("  <><>  [InterruptHandler] -------- [IMPLSENDINDICATION more than one nTxPending] <><> Error !"));
	//				NKDbgPrintfW(TEXT("  <><>  [InterruptHandler] -------- [IMPLSENDINDICATION more than one nTxPending] <><> Error !"));
	//				NKDbgPrintfW(TEXT("  <><>  [InterruptHandler] -------- [IMPLSENDINDICATION more than one nTxPending] <><> Error !"));
	//			}
  //#else
  //#endif
				while (Adapter->m_nTxPendings)
				{
				Adapter->m_nTxPendings--;
				REPORT(Adapter, TID_GEN_XMIT_OK, 1); 
				NdisMSendResourcesAvailable(Adapter->MiniportAdapterHandle);
				}
			}
	}
#elif (DM9000_IMPL_SEND==SEND_INT_TXEND_RSRC)
	
	.. xcv  v ds vl sdv .. mmkm k dv k kvaeajj,m m md sv..
	// return if not TX latch
	if(uValue & 0x02) { //~return; need-Device-Enable-Interrupt;

			nsr = DeviceReadPort(Adapter,DM9_NSR);
			
			// check TX-END2
			if(nsr & 0x08)
			{
				Adapter->m_nTxPendings--;
				REPORT(Adapter, TID_GEN_XMIT_OK, 1); 

		//#ifdef IMPL_SEND_INDICATION
			//	NdisMSendComplete(
			//			Adapter->MiniportAdapterHandle,
			//			(PNDIS_PACKET)(((PCQUEUE_GEN_HEADER)Adapter->uaddr)->pPacket),
			//			NDIS_STATUS_SUCCESS);
		//#endif
			
				NdisMSendResourcesAvailable(Adapter->MiniportAdapterHandle);
			}

			// check TX-END1
			if(nsr & 0x04)
			{
				Adapter->m_nTxPendings--;
				REPORT(Adapter, TID_GEN_XMIT_OK, 1);

		//#ifdef IMPL_SEND_INDICATION
			//	NdisMSendComplete(
			//			Adapter->MiniportAdapterHandle,
			//			(PNDIS_PACKET)(((PCQUEUE_GEN_HEADER)Adapter->uaddr)->pPacket),
			//			NDIS_STATUS_SUCCESS);
		//#endif
				NdisMSendResourcesAvailable(Adapter->MiniportAdapterHandle);
			}
	}
#endif
//todo
	DeviceEnableInterrupt(Adapter);
}

VOID
MiniportISRHandler(
    OUT PBOOLEAN InterruptRecognized,
    OUT PBOOLEAN QueueDpc,
    IN PVOID Context
    )
{
	U32 uValue;
	U32 uValue1;
    PNE2000_ADAPTER Adapter = (PNE2000_ADAPTER)Context;

//    NKDbgPrintfW(TEXT("[dm9] -- Jeffrey .. isr MiniportISRHandler\r\n"));
//	    NKDbgPrintfW(TEXT("[dm9] -- Jeffrey .. isr  .. MiniportISRHandler.. 1\r\n"));


//todo
    DeviceDisableInterrupt(Adapter);

//	    NKDbgPrintfW(TEXT("[dm9] -- Jeffrey .. isr  .. MiniportISRHandler.. 2\r\n"));

	*InterruptRecognized= FALSE;
	*QueueDpc= FALSE;
//	    NKDbgPrintfW(TEXT("[dm9] -- Jeffrey .. isr  .. MiniportISRHandler.. 3\r\n"));

	uValue= DeviceReadPort(Adapter, DM9_ISR);
	uValue1= DeviceReadPort(Adapter, DM9_IMR);
	
	
//j	    NKDbgPrintfW(TEXT("[dm9] -- Jeffrey .. isr  .. MiniportISRHandler.. 4 [ISR= %x, IMR= %x]\r\n", uValue,uValue1));
	Adapter->m_uRecentInterruptStatus= uValue;
	
/*    .NKDbgPrintfW((TEXT("[dm9] -- ----------------- ISR. [ISRHandler] -------- [ISR= %x]\r\n"), uValue)); */
//    NKDbgPrintfW(TEXT("[dm9] -- Jeffrey .. isr\r\n"));
	
	//ISRPrint(TEXT("  <><>  [ISRHandler] -------- [ISR= %x]"), ARGS(uValue));
	//if (uValue&0x01) ISRPrint(TEXT("[rx]"));
	//if (uValue&0x02) ISRPrint(TEXT("[tx]"));
	
//	    NKDbgPrintfW(TEXT("[dm9] -- Jeffrey .. isr  .. MiniportISRHandler.. 5\r\n"));
	if (uValue & 0x3f) //((MAKE_MASK4(3,2,1,0))
	{
//	    NKDbgPrintfW(TEXT("[dm9] -- Jeffrey .. isr  .. MiniportISRHandler.. 6\r\n"));
		*InterruptRecognized= TRUE;
		*QueueDpc= TRUE;

		/* clear it immediately */
		DeviceWritePort(Adapter, DM9_ISR, uValue);
		//ISRPrint(TEXT(" ----------"));
		//ISRPrint(TEXT("\r\n"));
	}
	else
//todo
	{
//	    NKDbgPrintfW(TEXT("[dm9] -- Jeffrey .. isr  .. MiniportISRHandler.. 7\r\n"));
		//ISRPrint(TEXT("\r\n"));
		DeviceEnableInterrupt(Adapter);
	}
//	    NKDbgPrintfW(TEXT("[dm9] -- Jeffrey .. isr  .. MiniportISRHandler.. 8\r\n"));
}

NDIS_STATUS
MiniportQueryInformation(
    IN NDIS_HANDLE MiniportAdapterContext,
    IN NDIS_OID Oid,
    IN PVOID InformationBuffer,
    IN ULONG InformationBufferLength,
    OUT PULONG BytesWritten,
    OUT PULONG BytesNeeded
)
{
	int 	n;
	PVOID	panswer;
	U8		szbuffer[32];
    PNE2000_ADAPTER Adapter = (PNE2000_ADAPTER)MiniportAdapterContext;
    //NDIS_STATUS StatusToReturn = NDIS_STATUS_SUCCESS;
	NDIS_STATUS s= NDIS_STATUS_SUCCESS;
	
	//(*******dbg)
	if (FindEverQOID(Adapter, Oid))
		n= Adapter->m_QFindIndex;
	else
		n= Adapter->nqOID++;
	Adapter->m_qOID[n]= Oid;
	Adapter->ct_qOID[n]++;
	
	//QPrint(TEXT("   [QueryInformation] - [dm9] Oid= %08x --"), Oid);

#if 1
    if (Oid==OID_802_3_PERMANENT_ADDRESS){
    	Adapter->ct_qOID[n] |= 0x8000;
    	//HANDLE_QUERY( OID_802_3_PERMANENT_ADDRESS,
		//	DeviceMacAddress(&szbuffer[0]),ETH_ADDRESS_LENGTH);
		//  =	 
		if(InformationBufferLength < (*BytesNeeded= ETH_ADDRESS_LENGTH)) 
		{
			s = NDIS_STATUS_INVALID_LENGTH; 
			TO_PRINT(TEXT("[dm9] PERMANENT_ADR, NDIS_STATUS_INVALID_LENGTH\r\n"));
			return s; 
		}
		panswer = (void*) DeviceMacAddress(Adapter, &szbuffer[0]); 
		/*NKDbgPrintfW(TEXT("\r\n[dm9PermanentAddrQuery]= %02x %02x %02x %02x %02x %02x ..\r\n"), 
				szbuffer[0],
				szbuffer[1],
				szbuffer[2],
				szbuffer[3],
				szbuffer[4],
				szbuffer[5]
				);*/
		*BytesWritten = ETH_ADDRESS_LENGTH;
		
		PrintMoveMem(Oid, *BytesWritten, (PU8)panswer ); //dbg
		NdisMoveMemory(InformationBuffer, panswer,*BytesWritten);
		//return s; 
		//QPrint(TEXT("  Process  %x "), Oid);
    }
#endif    
    if (Oid==OID_802_3_CURRENT_ADDRESS){
		Adapter->ct_qOID[n] |= 0x8000;
    	//HANDLE_QUERY( OID_802_3_CURRENT_ADDRESS,
		//	DeviceMacAddress(&szbuffer[0]),ETH_ADDRESS_LENGTH);
		//  =	
		if(InformationBufferLength < (*BytesNeeded= ETH_ADDRESS_LENGTH)) 
		{ 
		  s = NDIS_STATUS_INVALID_LENGTH; 
		  TO_PRINT(TEXT("[dm9] CURRENT_ADR, NDIS_STATUS_INVALID_LENGTH\r\n"));
		  return s; 
		}
		panswer = (void*) DeviceMacAddress(Adapter, &szbuffer[0]); 
		/*NKDbgPrintfW(TEXT("\r\n[dm9MacQuery]= %02x %02x %02x %02x %02x %02x ..\r\n"), 
				szbuffer[0],
				szbuffer[1],
				szbuffer[2],
				szbuffer[3],
				szbuffer[4],
				szbuffer[5]
				);*/
		*BytesWritten = ETH_ADDRESS_LENGTH; 
		
		PrintMoveMem(Oid, *BytesWritten, (PU8)panswer ); //dbg
		NdisMoveMemory(InformationBuffer, panswer,*BytesWritten);
		//return s; 
		//QPrint(TEXT("  Process  %x "), Oid);
    }
    if (Oid==OID_GEN_MEDIA_CONNECT_STATUS)
    {
		Adapter->ct_qOID[n] |= 0x8000;
		//HANDLE_QUERY( OID_GEN_MEDIA_CONNECT_STATUS,
		//	&m_pLower->m_szCurrentSettings[SID_MEDIA_CONNECTION_STATUS],sizeof(U32));
		//  =	
		if(InformationBufferLength < (*BytesNeeded= sizeof(U32))) 
		{ 
		  s = NDIS_STATUS_INVALID_LENGTH; 
		  return s; 
		}
		panswer = (void*)&Adapter->m_szCurrentSettings[SID_MEDIA_CONNECTION_STATUS]; 
		*BytesWritten = sizeof(U32); 

		PrintMoveMem(Oid, *BytesWritten, (PU8)panswer ); //dbg
		NdisMoveMemory(InformationBuffer, panswer,*BytesWritten);
		//return s; 
		//QPrint(TEXT("  Process  %x "), Oid);
		//QPrint(TEXT("\r\n"));
		if (Adapter->m_szCurrentSettings[SID_MEDIA_CONNECTION_STATUS]==NdisMediaStateConnected)
			QPrint(TEXT("  < Processing(Recheck):Query= NdisMediaStateConnected > "));
		else
			QPrint(TEXT("  < Processing(Recheck):Query= NdisMediaStateDisconnected > "));
    }
    if (Oid==OID_802_3_MULTICAST_LIST) // 'Not be callled'
    {
		Adapter->ct_qOID[n] |= 0x8000;
		//HANDLE_QUERY( OID_802_3_MULTICAST_LIST,
 		//	&m_pLower->m_szMulticastList[0][0],
 		//	m_pLower->m_nMulticasts*ETH_ADDRESS_LENGTH);
		if(InformationBufferLength < (*BytesNeeded= Adapter->m_nMulticasts*ETH_ADDRESS_LENGTH)) 
		{ 
		  s = NDIS_STATUS_INVALID_LENGTH; 
		  return s; 
		}
		panswer = (void*)&Adapter->m_szMulticastList[0][0]; 
		*BytesWritten = *BytesNeeded; 
		
		PrintMoveMem(Oid, *BytesWritten, (PU8)panswer ); //dbg
		NdisMoveMemory(InformationBuffer, panswer,*BytesWritten);
		//return s; 
		//QPrint(TEXT("  Process  %x "), Oid);
    }
    
	s= DriverQueryInformation(
			Adapter, n,
			Oid,
			InformationBuffer, 
			InformationBufferLength, 
			BytesWritten,
			BytesNeeded);

	//QPrint(TEXT("\r\n"));

    return s;
}

NDIS_STATUS
MiniportReset(
    OUT PBOOLEAN AddressingReset,
    IN NDIS_HANDLE MiniportAdapterContext
    )
{
    PNE2000_ADAPTER Adapter = (PNE2000_ADAPTER)MiniportAdapterContext;
    NKDbgPrintfW(TEXT("   [MiniportReset] -------- [dm9] 2011 ------------------------ \r\n"));
	//NKDbgPrintfW(TEXT("     Reset=  Reset --------------------------------\r\n"));
    return DriverReset(Adapter, AddressingReset);
}    

NDIS_STATUS
MiniportSend(
    IN NDIS_HANDLE MiniportAdapterContext,
    IN PNDIS_PACKET Packet,
    IN UINT Flags
    )
{
	U32 wait= 0;
	int iCont1=1;		//&*&*&J1_add
    PNE2000_ADAPTER Adapter = (PNE2000_ADAPTER)MiniportAdapterContext;
	//TXPrint(TEXT("   [MiniportSend] -------- [dm9] 2011 ------------------------ \r\n"));

#if (DM9000_IMPL_SEND==SEND_INT_TXEND_RSRC)
	
	...vc, ,b, bdf,.df .. mmfcdmn jksfswvwjjjdkjnjknsr..
	if (!(Adapter->m_nTxPendings < Adapter->m_nMaxTxPending))
	{
			return NDIS_STATUS_RESOURCES;
	}
#endif

//#ifdef IMPL_SEND_INDICATION
  //  ...
  //	if (!QueryPacket(Adapter, Packet, Flags)) return NDIS_STATUS_FAILURE;
  //  ...
//#else
//#endif
	
	//DeviceSend();
	//=
	Adapter->QNDIS_State= 0;
	QueryPacket_NDIS(Adapter, Packet, Flags);

  #if (DM9000_IMPL_SEND==SEND_WAIT_TXCMPLT)
	//(1)if (Adapter->m_nTxPendings >= Adapter->m_nMaxTxPending)
	//(2)if (Adapter->m_nTxPendings)
	if (Adapter->m_nTxPendings >= Adapter->m_nMaxTxPending)
	{
		while (iCont1) //1)		//&*&*&J1_mod
		{
			if (DeviceReadPort(Adapter, 0x02) & 0x01)
			{
				wait++;
				if (!QueryPacket_NDIS(Adapter, Packet, Flags)) return NDIS_STATUS_FAILURE;
			}
			else
			{
				while (Adapter->m_nTxPendings)
				{
				Adapter->m_nTxPendings--;
				REPORT(Adapter, TID_GEN_XMIT_OK, 1); 
				NdisMSendResourcesAvailable(Adapter->MiniportAdapterHandle); // more times called is no problem.
				}
				break;
			}
		}
		
		if (wait)
		{
			Adapter->txw += wait;
			Adapter->txwno++;
			if (Adapter->txwno==TXNO_STAND_NO)  // 2000, 15
			{
				wait= Adapter->txw / Adapter->txwno;
				NKDbgPrintfW(TEXT("[MaxTxPdng- %d] wait *%d\r\n"), Adapter->m_nMaxTxPending, wait); // ThreeCntrPrint
				Adapter->txw= Adapter->txwno= 0;
			}
		}
	}
  #endif
	while (Adapter->QNDIS_State==2) { // if ever be 0, will always return failure, if still not be 1, must only be 2. 
		if (!QueryPacket_NDIS(Adapter, Packet, Flags)) return NDIS_STATUS_FAILURE;
	}
	if (!QueryPacket_SEND(Adapter)) return NDIS_STATUS_FAILURE;
	/*
	if (!QueryPacket_DM9(Adapter, Packet, Flags)) return NDIS_STATUS_FAILURE;  // this func try to save 1 memcpy.
	Adapter->m_nTxPendings++;
	*/
	PrintTxHead(14, (PU8)CQueueGetUserPointer(Adapter->uaddr)); //(PCQUEUE_GEN_HEADER)Adapter->uaddr
	//=
	//pcurr= (PCQUEUE_GEN_HEADER)Adapter->uaddr;
	//szbuffer= CQueueGetUserPointer(pcurr);
	//TXPrint(TEXT("[dm9tx]= %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x  ..\r\n"), 
	//	    szbuffer[0],
	//	    szbuffer[1],
	//	    szbuffer[2],
	//	    szbuffer[3],
	//	    szbuffer[4],
	//	    szbuffer[5],
	//	    szbuffer[6],
	//	    szbuffer[7],
	//	    szbuffer[8],
	//	    szbuffer[9],
	//	    szbuffer[10],
	//	    szbuffer[11],
	//	    szbuffer[12],
	//	    szbuffer[13]
	//		);

	//SendSetFlag(Adapter);
	
//#ifdef IMPL_SEND_INDICATION
  //  return NDIS_STATUS_PENDING; // (Cancel)
//#endif
	return NDIS_STATUS_SUCCESS;
}

extern
NDIS_STATUS
MiniportSetInformation(
    IN NDIS_HANDLE MiniportAdapterContext,
    IN NDIS_OID Oid,
    IN PVOID InformationBuffer,
    IN ULONG InformationBufferLength,
    OUT PULONG BytesRead,
    OUT PULONG BytesNeeded
    )
{	
	int n;
    PNE2000_ADAPTER Adapter = (PNE2000_ADAPTER)MiniportAdapterContext;
	NDIS_STATUS	s= NDIS_STATUS_SUCCESS;
	
	//(*******dbg)
	if (FindEverSOID(Adapter, Oid))
		n= Adapter->m_SFindIndex;
	else
		n= Adapter->nsOID++;
	Adapter->m_sOID[n]= Oid;
	Adapter->ct_sOID[n]++;
	
    QPrint(TEXT("   [SetInformation] -------- [dm9] ------------------------ \r\n"));
	//QPrint(TEXT("     Oid=  %d ---- "), Oid);

	switch (Oid)
	{
	  //HANDLE_SET( OID_GEN_CURRENT_PACKET_FILTER,sizeof(U32));
	  //=	
		case OID_GEN_CURRENT_PACKET_FILTER: 
			Adapter->ct_sOID[n] |= 0x8000;
			QPrint(TEXT(" Process:%x ---- "), Oid);
		    if(InformationBufferLength < (*BytesNeeded=sizeof(U32)))
			{ 
				s = NDIS_STATUS_INVALID_LENGTH; 
				break; 
			}	
			*BytesRead = sizeof(U32); 
			NKDbgPrintfW(TEXT(" [DM9]SetInfo: GEN_CURRENT_PACKET_FILTER, uFilter= %x ---- \r\n"), *(U32*)InformationBuffer);
			
			DeviceOnSetupFilter_1(Adapter, *(U32*)InformationBuffer); 
			break;
		
		case OID_802_3_MULTICAST_LIST:
			OnceSetInfoFlag(Adapter); // "dbg"

			Adapter->ct_sOID[n] |= 0x8000;
			//QPrint(TEXT(" Process:%x ---- "), Oid);
			NdisMoveMemory(
				&Adapter->m_szMulticastList[0][0],
				InformationBuffer,
				InformationBufferLength);
				
			Adapter->m_nMulticasts = InformationBufferLength / ETH_ADDRESS_LENGTH;
			NKDbgPrintfW(TEXT(" [DM9]SetInfo: 802_3_MULTICAST_LIST: uFilter= %x ---- \r\n"), 
				Adapter->m_szCurrentSettings[SID_GEN_CURRENT_PACKET_FILTER] | NDIS_PACKET_TYPE_MULTICAST);
			NKDbgPrintfW(TEXT(" [DM9]SetInfo: nMulticasts= %d ---- \r\n"), Adapter->m_nMulticasts);
				
			DeviceOnSetupFilter_1(
				Adapter,
				Adapter->m_szCurrentSettings[SID_GEN_CURRENT_PACKET_FILTER] | NDIS_PACKET_TYPE_MULTICAST);
			break;

		// don't care oids
		case OID_GEN_CURRENT_LOOKAHEAD:
			Adapter->ct_sOID[n] |= 0x8000;
			break;

		case OID_GEN_NETWORK_LAYER_ADDRESSES:
		default:
			s = NDIS_STATUS_INVALID_OID;
		
	} // of switch Oid
	//QPrint(TEXT("\r\n"));
	return s;
}    


NDIS_STATUS
Ne2000TransferData(
    OUT PNDIS_PACKET InPacket,
    OUT PUINT BytesTransferred,
    IN NDIS_HANDLE MiniportAdapterContext,
    IN NDIS_HANDLE MiniportReceiveContext,
    IN UINT ByteOffset,
    IN UINT BytesToTransfer
    )
{
	//NKDbgPrintfW(TEXT("   ?[Ne2000TransferData] -------- [dm9] 2011 ------------------------ ? \r\n"));
	//NKDbgPrintfW(TEXT("   ?[Ne2000TransferData] -------- [dm9] 2011 ------------------------ ? \r\n"));
	//NKDbgPrintfW(TEXT("   ?[Ne2000TransferData] -------- [dm9] 2011 ------------------------ ? \r\n"));
	return(NDIS_STATUS_SUCCESS);
}    