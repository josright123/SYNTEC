
#include "precomp.h"

void EDeviceInitialize(PNE2000_ADAPTER Adapter)
{
	U32		val;
	int n;
	U8 sz[10];
	U8 rdMAC[6];
	
//	DEBUG_PRINT((TEXT("[dm9] E.Device.Initialize=%d\r\n"),nResetCounts));
	
	// reset member varialbes
//	m_uLastAddressPort = (U32)-1;
	
	// software reset the device
	DeviceWritePort(Adapter, DM9_NCR, 0x03); //[E.Device.Initialize] MAC_Intl_Lpbk+SwReset
	NdisStallExecution(20);

	DeviceWritePort(Adapter, DM9_NCR, 0x03);
	NdisStallExecution(20);
	//NKDbgPrintfW(TEXT("[dm9] -- %02x -- \r\n"), DeviceReadPort(Adapter, DM9_NCR)); // was 0x00

	//DeviceWritePort(DM9_NCR, 0x00);
	
	// read the io orgnization
	// ISR<7:6> == x1, dword  //32-bit
	// ISR<7:6> == 0x, word   //16-bit
	// ISR<7:6> == 10, byte mode   //8-bit
	val = DeviceReadPort(Adapter, DM9_ISR);
	if(val & MAKE_MASK(6))
	{
		Adapter->m_nIoMode = DWORD_MODE;
		Adapter->m_nIoMaxPad = 3;
	}
	else if(!(val & MAKE_MASK(7)))
	{
		Adapter->m_nIoMode = WORD_MODE;
		Adapter->m_nIoMaxPad = 1;
	}
	else
	{
		Adapter->m_nIoMode = BYTE_MODE;
		Adapter->m_nIoMaxPad = 0;
	}
	/*if(val & 0x80)
		NKDbgPrintfW((TEXT("[dm9] -- ----------------- ISR.b[7] = 1 (8-bit) -- \r\n")));
	else
		NKDbgPrintfW((TEXT("[dm9] -- ----------------- ISR.b[7] = 0 (16-bit) -- \r\n")));*/
	NKDbgPrintfW(TEXT("[dm9] -- EDevice.Initialize IOMOD : ISR= %02x -- \r\n"), val);
	
	// activate internal phy
	DeviceWritePort(Adapter, DM9_GPR,  0x00);
	NdisStallExecution(2000); // 2000 us //mm2_EnhanceLink
	// select GPIO 0<0>, set it to output
	DeviceWritePort(Adapter, DM9_GPCR, (1<<0));
	// output zero to activate internal phy
	DeviceWritePort(Adapter, DM9_GPR,  0x00);
	NdisStallExecution(2000); // 2000 us //mm2_EnhanceLink
	
	val = DeviceReadPort(Adapter, DM9_GPR);
	NKDbgPrintfW(TEXT("[dm9] -- EDevice.Initialize Get : GPR= %02x -- (#bit-0 check, =0 PHY-ON) \r\n"), val);
	// clear TX status
	DeviceWritePort(Adapter, DM9_NSR, 0x00);
	
	//[dtri: High Active]
	DeviceWritePort(Adapter, 0x39, DM9000_CAST_INT);  //INT pin control register
	
	DeviceWritePort(Adapter, 0x2f, DM9000_CAST_SMODE_LINKPULSEE4); // smcr_version
	DeviceWritePort(Adapter, 0x38, DM9000_CAST_BUS_DRIVE | DM9000_CAST_BUS_SPIKE); // pbcr_version
	
	
   // Enable memory chain    ,DeviceWritePort(DM9_IMR, (1<<7))    ;ALREADY inside DeviceEnableInterrupt    
   DeviceWritePhy(Adapter, MIIADDR_DSP_CONTROL, 0xE100); //mm1 for speed LED issue

//--------------------------------------------------
	for(n=0;n<ETH_ADDRESS_LENGTH;n++)
	{
		rdMAC[n]= (U8) DeviceReadPort(Adapter, DM9_PAR0+n);
	}

	NKDbgPrintfW(TEXT("[pre-dm9MacRdReg]= %02x %02x %02x %02x %02x %02x ..\r\n"),
		rdMAC[0],
		rdMAC[1],
		rdMAC[2],
		rdMAC[3],
		rdMAC[4],
		rdMAC[5]
		);
			
	/* 1. set unicast */
	// retrive node address
	DeviceMacAddress(Adapter, &sz[0]);
	NKDbgPrintfW(TEXT("[dm9MacWrReg]= %02x %02x %02x %02x %02x %02x ..\r\n"), 
			sz[0],
			sz[1],
			sz[2],
			sz[3],
			sz[4],
			sz[5]
			);
	// set node address
	for(n=0;n<ETH_ADDRESS_LENGTH;n++)
		DeviceWritePort(Adapter, DM9_PAR0+n,(U32)sz[n]);
//--------------------------------------------------
	for(n=0;n<ETH_ADDRESS_LENGTH;n++)
	{
		rdMAC[n]= (U8) DeviceReadPort(Adapter, DM9_PAR0+n);
	}
	NKDbgPrintfW(TEXT("[dm9MacRdReg]= %02x %02x %02x %02x %02x %02x ..\r\n"),
			rdMAC[0],
			rdMAC[1],
			rdMAC[2],
			rdMAC[3],
			rdMAC[4],
			rdMAC[5]
			);
	if (rdMAC[0]!=sz[0] || 
	    rdMAC[1]!=sz[1] || 
	    rdMAC[2]!=sz[2] || 
	    rdMAC[3]!=sz[3] || 
	    rdMAC[4]!=sz[4] || 
	    rdMAC[5]!=sz[5])
	{
		NKDbgPrintfW (TEXT("[DM9]:-- *** Write MAC Registers *** --------------- !!! \r\n"));
		NKDbgPrintfW (TEXT("[DM9]:-- *** Write MAC Registers *** --Fail to write !!! \r\n"));
		NKDbgPrintfW (TEXT("[DM9]:-- *** Write MAC Registers *** --------------- !!! \r\n"));
	}

	NKDbgPrintfW(TEXT("[dm9: Chip signature is %08X]\r\n"), DeviceReadPort(Adapter, 0x29)<<24);
	Sleep(500);
//--------------------------------------------------

#if 0
#ifdef	IMPL_STORE_AND_INDICATION
	if(nResetCounts) return;
	// set indication timer 
	DeviceInitializeTimer();
#endif
#endif

// v3.2.9
	Adapter->m_nMaxTxPending = (DeviceReadPort(Adapter, DM9_CHIPREV) >= 0x10)?2:1;
	Adapter->m_nTxPendings = 0;
	Adapter->m_up= 0;
	//pwr-ptr
	//dbg
	Adapter->m_PwrUntillSend= Adapter->m_PwrUntillChkForHang= Adapter->m_PwrUntillOnceSetInfo= 0;

	//NKDbgPrintfW((TEXT("[dm9] -- ----------------- REG38.b[6:5] : CURR= 00 (2mA) -- \r\n")));
	//NKDbgPrintfW((TEXT("[dm9] -- ----------------- REG38.b[6:5] : CURR= 11 (8mA) -- \r\n")));
	val= DeviceReadPort(Adapter, 0x38);
	//NKDbgPrintfW (TEXT("[DM9]:-- -- CURR REG38 --- 0x%x -- CURR= %d%d\r\n"), val, val>>6&1, val>>5&1);
	//U32	nsr;
	val= DeviceReadPort(Adapter, 0x39);
	//NKDbgPrintfW (TEXT("[DM9]:-- --- INT REG39 --- 0x%x \r\n"), val);
	/*if (val &1) 
		NKDbgPrintfW (TEXT("[DM9]:-- *** INT REG39 *** INT Active low \r\n"));
	else
		NKDbgPrintfW (TEXT("[DM9]:-- *** INT REG39 *** INT Active high \r\n"));*/

	//DeviceEnableInterrupt();
}

NDIS_STATUS DriverReset(PNE2000_ADAPTER Adapter, PBOOLEAN	pbAddressingReset)
{
	Adapter->m_bSystemHang = 0;
	return NDIS_STATUS_SUCCESS;
}

void DeviceOnSetupFilter_0(PNE2000_ADAPTER Adapter)
{
	int		n;
	U8		sz[8];
	
	// prepare new hash table
	memset((void*)(&sz[0]),0,sizeof(sz));
	
	// if broadcast, its hash value is known as 63.
	// if(uFilter & NDIS_PACKET_TYPE_BROADCAST) sz[7] |= 0x80;
	   sz[7] |= 0x80;

	//if(uFilter & NDIS_PACKET_TYPE_MULTICAST)
	//	for(n=0;n<m_nMulticasts;n++)
	//	{
	//		hashval = DeviceCalculateCRC32(
	//			&m_szMulticastList[n][0],ETH_ADDRESS_LENGTH,FALSE) & 0x3f;
	//		sz[hashval/8] |= 1 << (hashval%8);
	//	} // of calculate hash table

	// submit the new hash table
	for(n=0;n<sizeof(sz);n++)
		DeviceWritePort(Adapter, DM9_MAR0+n,(U32)sz[n]);
	
	//DeviceWritePort(DM9_RXCR,m_szCurrentSettings[SID_OP_MODE]=newmode);	
	//DeviceWritePort(DM9_RXCR, 0x31);	
}


void DeviceOnSetupFilter_1(PNE2000_ADAPTER Adapter, U32 uFilter)
{
	int		n;
	U8		sz[8];
	U32		hashval;
	
	
	U32		newmode;
	// save old op mode
	newmode = Adapter->m_szCurrentSettings[SID_OP_MODE];
	// clear filter related bits,
	// pass all multicast<3> and promiscuous<1>
	newmode	&= ~MAKE_MASK2(3,1);
	
	// zero input means one reset request
	Adapter->m_szCurrentSettings[SID_GEN_CURRENT_PACKET_FILTER]= uFilter;
	NKDbgPrintfW(TEXT(" [DM9]SetInfo: Set uFilter= %x ---- \r\n"), uFilter);
	if(!(uFilter)) 
	{
		
		/* 1. set unicast */
		// retrive node address
		DeviceMacAddress(Adapter, &sz[0]);
		/*NKDbgPrintfW(TEXT("[seti-FILTER-WrReg]= %02x %02x %02x %02x %02x %02x ..\r\n"), 
				sz[0],
				sz[1],
				sz[2],
				sz[3],
				sz[4],
				sz[5]
				);*/
		// set node address
		for(n=0;n<ETH_ADDRESS_LENGTH;n++)
			DeviceWritePort(Adapter, DM9_PAR0+n,(U32)sz[n]);
			
		/* 2. clear multicast list and count */
		Adapter->m_nMulticasts = 0;
		memset((void*)&Adapter->m_szMulticastList,0,sizeof(Adapter->m_szMulticastList));
		NKDbgPrintfW(TEXT(" [DM9]SetInfo: Clear nMulticasts= %d ---- \r\n"), Adapter->m_nMulticasts);
		
		/* 3. clear hash table */
		// clear hash table
		memset((void*)(&sz[0]),0,sizeof(sz));
		for(n=0;n<sizeof(sz);n++)
			DeviceWritePort(Adapter, DM9_MAR0+n,(U32)sz[n]);

		//return uFilter;		
	}
	
	// if promiscuous mode<1> is requested,
	// just set this bit and return
	if( (uFilter & NDIS_PACKET_TYPE_PROMISCUOUS) )
	{
		// add promiscuous<1>
		newmode |= MAKE_MASK(1);
		DeviceWritePort(Adapter, DM9_RXCR, Adapter->m_szCurrentSettings[SID_OP_MODE]= newmode);
		//return uFilter;
	}
	

	// if pass all multicast<3> is requested,
	if(uFilter & NDIS_PACKET_TYPE_ALL_MULTICAST) newmode |= MAKE_MASK(3);
	
	
	
	// prepare new hash table
	memset((void*)(&sz[0]),0,sizeof(sz));
	
	// if broadcast, its hash value is known as 63.
	if(uFilter & NDIS_PACKET_TYPE_BROADCAST) sz[7] |= 0x80;

	if(uFilter & NDIS_PACKET_TYPE_MULTICAST)
	{
		/*NKDbgPrintfW(TEXT(" [DM9]SetInfo: IN=")); 
		NKDbgPrintfW(TEXT(" %02x"), sz[0]);
		NKDbgPrintfW(TEXT(" %02x"), sz[1]);
		NKDbgPrintfW(TEXT(" %02x"), sz[2]);
		NKDbgPrintfW(TEXT(" %02x"), sz[3]);
		NKDbgPrintfW(TEXT(" %02x"), sz[4]);
		NKDbgPrintfW(TEXT(" %02x"), sz[5]);
		NKDbgPrintfW(TEXT(" %02x"), sz[6]);
		NKDbgPrintfW(TEXT(" %02x"), sz[7]);
		NKDbgPrintfW(TEXT("\r\n"));*/
		for(n=0;n<Adapter->m_nMulticasts;n++)
		{
			//NKDbgPrintfW(TEXT(" [DM9] hash: add"));
			hashval = DeviceCalculateCRC32(
				&Adapter->m_szMulticastList[n][0], ETH_ADDRESS_LENGTH, FALSE) & 0x3f;
			sz[hashval/8] |= 1 << (hashval%8);
			//NKDbgPrintfW(TEXT(" --> sz[%d] |= 0x%02x"), hashval/8, 1 << (hashval%8));
			//NKDbgPrintfW(TEXT("\r\n"));
		} // of calculate hash table
	}

	// submit the new hash table
	//NKDbgPrintfW(TEXT(" [DM9]SetInfo: OUT=")); 
	for(n=0;n<sizeof(sz);n++)
	{
		//NKDbgPrintfW(TEXT(" %02x"), sz[n]);
		DeviceWritePort(Adapter,DM9_MAR0+n,(U32)sz[n]);
	}
	//NKDbgPrintfW(TEXT("\r\n"));
	
	DeviceWritePort(Adapter,DM9_RXCR, Adapter->m_szCurrentSettings[SID_OP_MODE]= newmode);
	
	//return uFilter;
}

void DeviceStart(PNE2000_ADAPTER Adapter)
{
#if 1
#if 0
//#ifdef IMPL_FLOW_CONTROL	
	U32		val;
	
	// set PHY supports flow control
	DeviceWritePhy(Adapter, 0, 4, (DeviceReadPhy(0,4)|(1<<10)));
	
	// check full-duplex mode or not<3>
	val = DeviceReadPort(Adapter, DM9_NCR);
	if( val & MAKE_MASK(3))
	{
		/* full duplex mode */
		val = DeviceReadPort(Adapter, DM9_PAUSETH);
		DeviceWritePort(Adapter, DM9_PAUSETH,(U8)val);
		
		// enable flow control<0>
		// enable pause packet<5>
		DeviceWritePort(Adapter, DM9_FLOW,MAKE_MASK2(5,0));
	}
	else
	{
		/* non full duplex mode */
		val = DeviceReadPort(Adapter, DM9_BACKTH);
		DeviceWritePort(Adapter, DM9_BACKTH, (U8)val);

		// enable back pressure<half dumplex mode)<4,3>
		DeviceWritePort(Adapter, DM9_FLOW,MAKE_MASK2(4,3));
	}
//#endif
#endif
#endif

	 // Copy from UTV210-DM9000A, set PHY supports flow control (通知對方) [Not check yet]
	 //DeviceWritePhy(0, 4, (DeviceReadPhy(0,4)|(1<<10)));
	 // Copy from V208, set PHY supports flow control (通知對方)
	 //DeviceWritePhy(0, 4, 0x05e1);
	 // Sel=
	 DeviceWritePhy(Adapter, 4, 0x05e1);
	 DeviceWritePort(Adapter, DM9_FLOW, 0x29);

	// enable interrupt
//doit
	 DeviceEnableInterrupt(Adapter);

//	 DeviceWritePort(Adapter, DM9_RXCR, Adapter->m_szCurrentSettings[SID_OP_MODE]);  // Write(DM9_RXCR, 0x31);	

	 NKDbgPrintfW(TEXT("[dm9]= DeviceStart RXCR=(%x)\r\n"), Adapter->m_szCurrentSettings[SID_OP_MODE]);

//dbg	 Adapter->m_szCurrentSettings[SID_OP_MODE] |= 0x02;
//dbg	 NKDbgPrintfW(TEXT("[dm9]= Test-ReAssign-Device.Start RXCR=(%x)\r\n"), Adapter->m_szCurrentSettings[SID_OP_MODE]);

	 DeviceWritePort(Adapter, DM9_RXCR, Adapter->m_szCurrentSettings[SID_OP_MODE]);
}

//"ndis"
//typedef	enum {
//	NIC_IND_TX_IDLE=0,
//	AID_ERROR=0x8000,
//	AID_LARGE_INCOME_PACKET,
//} NIC_IND_TYPE;			
//void DriverIndication(U32 uIndication)
//{
//	switch (uIndication)
//	{
//		case NIC_IND_TX_IDLE:
//			if(m_bOutofResources)
//			{
//				m_bOutofResources = 0;
//				NdisMSendResourcesAvailable(Adapter->MiniportAdapterHandle);
//			}
//			break;		
//		case AID_ERROR:
//		case AID_LARGE_INCOME_PACKET:
//			m_bSystemHang = 1;
//		default:
//			break;
//	} // of switch
//}

/*
#define RX_PKTS_ONPROC	6          ;  try to constrain it to recieve max 
int		rxpkts=0;          ;  6 packets once enter Dm9 Lookup Rx bufers
for(;rxpkts<RX_PKTS_ONPROC;){      ;
		rxpkts++;          ;
}                                  ;
*/

#define DM9051_RWPAL (0x24)
#define DM9051_RWPAH (0x25)
#define DM9051_MRRL (0xf4)
#define DM9051_MRRH (0xf5)
#define RX_BUFFER_START       0xC00
#define RX_BUFFER_END         0x4000
#define MONITOR_RXPOINTER_PACKET_NUM	8	// log times used in dbg mode (log from power-up)
#define MONITOR_RXPOINTER_CYCLE_NUM		3	// log times used in dbg mode (log for recycle happen)
typedef U16 uint16_t;
typedef U32 uint32_t;

uint16_t wrpadiff(uint16_t rwpa_s, uint16_t rwpa_e)
{
    return (rwpa_e >= rwpa_s) ? 
           rwpa_e - rwpa_s : 
           (rwpa_e + RX_BUFFER_END - RX_BUFFER_START) - rwpa_s;
}

/**
 * @brief  Read RX write pointer and memory data read address
 * 
 * @param  rwpa_wt  Pointer to store write pointer
 * @param  mdra_rd  Pointer to store read address
 */
#if 0
void dm9051_read_rx_pointers(PNE2000_ADAPTER Adapter, uint16_t *rwpa_wt, uint16_t *mdra_rd) 
{
  //*rwpa_wt = (uint16_t)DeviceReadPort(Adapter, DM9051_RWPAL) |
  //           (uint16_t)DeviceReadPort(Adapter, DM9051_RWPAH) << 8;
  //*mdra_rd = (uint16_t)DeviceReadPort(Adapter, DM9051_MRRL) |	
  //           (uint16_t)DeviceReadPort(Adapter, DM9051_MRRH) << 8;
    U32	val;
	val  = DeviceReadPort(Adapter, DM9051_RWPAL);
	val |= DeviceReadPort(Adapter, DM9051_RWPAH)<<8;
	*rwpa_wt = val;
	val  = DeviceReadPort(Adapter, DM9051_MRRL);
	val |= DeviceReadPort(Adapter, DM9051_MRRH)<<8;
	*mdra_rd = val;
}
#endif

static uint16_t DM_ETH_ToCalc_rx_pointers(PNE2000_ADAPTER Adapter, int state, const uint16_t *mdra_rd_org, uint16_t *mdra_rd_now)
{
//uint16_t dummy_rwpa;
//  dm9051_read_rx_pointers(Adapter, &dummy_rwpa, mdra_rd_now);
  //debug_diff_rx_pointers(state, *mdra_rd_now);
  return (state == 0) ? 0 : wrpadiff(*mdra_rd_org, *mdra_rd_now);
}

/* debug */
static void diff_rx_pointers_s(PNE2000_ADAPTER Adapter, uint16_t *pMdra_rds) {
	uint16_t dummy_rds= 0xc00;
	DM_ETH_ToCalc_rx_pointers(Adapter, 0, &dummy_rds, pMdra_rds);
}

static int fifoTurn_nRx = 0; //...
static fifoCyc_num = 0, fifoPkt_num = 0; //...

static void diff_rx_pointers_e(PNE2000_ADAPTER Adapter, uint16_t *pMdra_rds, uint32_t mdra_rd_now) {
	uint16_t rwpa_wt;
	if (fifoCyc_num < MONITOR_RXPOINTER_CYCLE_NUM ||
		fifoPkt_num < MONITOR_RXPOINTER_PACKET_NUM) {
		static uint16_t diffmdra_rd = 0x4000;
		static uint16_t dfmdra_rd = 0xc00;
		uint16_t mdra_rd = (uint16_t) mdra_rd_now;
		uint16_t diff = DM_ETH_ToCalc_rx_pointers(Adapter, 1, pMdra_rds, &mdra_rd); //......................
		if (mdra_rd < diffmdra_rd && (diffmdra_rd != 0x4000)) {
			diff += (mdra_rd >= *pMdra_rds) ? 0x3400 : 0;
			
			/* cycle-time */
			fifoCyc_num++;
			
  rwpa_wt = (uint16_t)DeviceReadPort(Adapter, DM9051_RWPAL) |
             (uint16_t)DeviceReadPort(Adapter, DM9051_RWPAH) << 8;
             
			NKDbgPrintfW(TEXT("[dbg_cycs %d.%d] mdra.s %02x%02x e %02x%02x dif %02x%02x (nrx %d) NET.rwpa_wt %02x%02x\r\n"),
				MONITOR_RXPOINTER_CYCLE_NUM, fifoCyc_num,
				*pMdra_rds >> 8, *pMdra_rds & 0xff,
				mdra_rd >> 8, mdra_rd & 0xff,
				diff >> 8, diff & 0xff,
				fifoTurn_nRx,
				rwpa_wt>> 8, rwpa_wt & 0xff);
			fifoTurn_nRx = 0;
	#if 1 //[temp store]
			fifoPkt_num = MONITOR_RXPOINTER_PACKET_NUM + MONITOR_RXPOINTER_PACKET_NUM;
	#endif
		}
		if (fifoPkt_num < MONITOR_RXPOINTER_PACKET_NUM) {
			uint16_t df = wrpadiff(dfmdra_rd, mdra_rd);
			fifoPkt_num++;
			
  rwpa_wt = (uint16_t)DeviceReadPort(Adapter, DM9051_RWPAL) |
             (uint16_t)DeviceReadPort(Adapter, DM9051_RWPAH) << 8;
			
			NKDbgPrintfW(TEXT("[dbg_pkts %d.%d] mdra.s %02x%02x e %02x%02x dif %02x%02x (len %d nrx %d) NET.rwpa_wt %02x%02x\r\n"),
				MONITOR_RXPOINTER_PACKET_NUM, fifoPkt_num,
				*pMdra_rds >> 8, *pMdra_rds & 0xff,
				mdra_rd >> 8, mdra_rd & 0xff,
				diff >> 8, diff & 0xff,
				df,
				fifoTurn_nRx,
				rwpa_wt>> 8, rwpa_wt & 0xff);
		}
		dfmdra_rd = mdra_rd;
		diffmdra_rd = mdra_rd;
	} //do .. while(0);

	#if 1
	if (fifoPkt_num == (MONITOR_RXPOINTER_PACKET_NUM + MONITOR_RXPOINTER_PACKET_NUM))
		fifoPkt_num = 0;
	if (fifoCyc_num >= MONITOR_RXPOINTER_CYCLE_NUM)
		fifoCyc_num = 0;
	#endif
}

#if 1
uint16_t gkeep_mdra_rds;

void diff_rx_s(PNE2000_ADAPTER Adapter)
{
	if (!fifoTurn_nRx) {
		gkeep_mdra_rds = DeviceReadPort(Adapter, 0xf4);
		gkeep_mdra_rds |= DeviceReadPort(Adapter, 0xf5) << 8;
		diff_rx_pointers_s(Adapter, &gkeep_mdra_rds); //&mdra_rds
	}
}

void diff_rx_e(PNE2000_ADAPTER Adapter)
{
	uint32_t mdra_rd_now;
	fifoTurn_nRx++;
		mdra_rd_now = DeviceReadPort(Adapter, 0xf4);
		mdra_rd_now |= DeviceReadPort(Adapter, 0xf5) << 8;
	diff_rx_pointers_e(Adapter, &gkeep_mdra_rds, mdra_rd_now); //&mdra_rds
}
#endif

int PrintBLine(U8 *p, int tlen, int len) {
	//int n = len;
	if (len >= 16) {
			NKDbgPrintfW(TEXT("[dm9] tlen %d, Each RxDat === %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\r\n"),
				tlen, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], 
				p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
	} else {
			int i, nfinal = len;
			if (len > 8) {
				NKDbgPrintfW(TEXT("[dm9] Each RxDat === %02x %02x %02x %02x %02x %02x %02x %02x \r\n"),
					p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
				nfinal -= 8;
				p += 8;
				//[.]
			}
			//[.]
			NKDbgPrintfW(TEXT("[dm9] Each RxDat ===\r\n"));
			for (i = 0; i< nfinal; i++) {
				NKDbgPrintfW(TEXT("+%02x\r\n"),p[i]);
			}
			NKDbgPrintfW(TEXT("\r\n"));
	}
	return len >= 16 ? 16 : len; //return n;
}

void Dm9LookupRxBuffers(PNE2000_ADAPTER Adapter)
{
	U32		desc;
	PDM9_RX_DESCRIPTOR pdesc;
	U32		value;
	U8 szbuffer[DRIVER_BUFFER_SIZE];  //= 1520
	int		counts=0;
	int		errors=0;

	diff_rx_s(Adapter);

	for(pdesc=(PDM9_RX_DESCRIPTOR)&desc;;)
	{
		//CHECK_SHUTDOWN(); //=if(m_bShutdown) break

		// probe first byte
		desc= DeviceReadDataWithoutIncrement(Adapter);
	
		// check if packet available, 01h means available, 00h means no data
		if(pdesc->bState != 0x01) break;

		// get the data descriptor again (DeviceReadData()= )
		desc= *(PU32)DeviceReadString(Adapter, (PU8)&value, sizeof(value));

		// read out the data to buffer
		// Performance issue: maybe we may discard the data
		// just add the rx address.
		// if the length is greater than buffer size, ...
		if((pdesc->nLength > DRIVER_BUFFER_SIZE)) //=0x5F0  //= 1520
		{
			//DriverIndication(AID_LARGE_INCOME_PACKET);
			//= 
			Adapter->m_bSystemHang = 1;
			break;
		}

		DeviceReadString(Adapter, (PU8)&szbuffer, pdesc->nLength);

		Adapter->nLen = pdesc->nLength;
		Adapter->headVal = value;
		memcpy(Adapter->szbuff, (PU8)&szbuffer, Adapter->nLen); //memcpy
#if 1
	PrintBLine(Adapter->szbuff, Adapter->nLen, 16);
#endif
		
//		diff_rx_s(Adapter); //.
		diff_rx_e(Adapter);

		// check status, as specified in DM9_RXSR,
		// the following bits are error
		// <3> PLE
		// <2> AE
		// <1> CE
		// <0> FOE
		if(pdesc->bStatus & MAKE_MASK4(3,2,1,0))
		{
			errors++;
			continue;
		} // of error happens

		counts++;
	 //ndis
		RXPrint(TEXT("[dm9rx]= %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x  ..\r\n"), 
			    szbuffer[0],
			    szbuffer[1],
			    szbuffer[2],
			    szbuffer[3],
			    szbuffer[4],
			    szbuffer[5],
			    szbuffer[6],
			    szbuffer[7],
			    szbuffer[8],
			    szbuffer[9],
			    szbuffer[10],
			    szbuffer[11],
			    szbuffer[12],
			    szbuffer[13]
				);
		DriverReceiveIndication(
			Adapter,
			0,
			(PVOID)&szbuffer,
			pdesc->nLength);
		
		Adapter->rp++;
	} // of forever read loop
	Adapter->rpno++;

	if (Adapter->rpno==RPNO_STAND_NO)  // 2000, 15
	{
		NKDbgPrintfW(TEXT("[DM9-LookupRx] %d for %d\r\n"), Adapter->rp, Adapter->rpno);  // ThreeCntrPrint
		Adapter->rp= Adapter->rpno= 0;
	}

//special
#ifdef	IMPL_STATISTICS
	REPORT(Adapter,TID_GEN_RCV_OK, counts);
	REPORT(Adapter,TID_GEN_RCV_ERROR, errors);
#endif
	//return counts;
}
