
#include <Ndis.h>
#include <PKFuncs.h>
#include "LPC3250_NDIS_Adapter.h"
#include "LPC3250_NDIS_Proto.h"
#include  "clkpwr_support.h"
#include "Bsp.h"
//#include "lpcddk.h"



#define IS_ALPHANUM(_x)		(  ((_x >= 'a') && (_x <= 'z')) \
							|| ((_x >= 'A') && (_x <= 'Z')) \
							|| ((_x >= '0') && (_x <= '9')) )

void LPC_PrintFrame(BYTE *pbData, DWORD dwLength)
{
	int a,b;

	DEBUGMSG(ZONE_INFO, (L"\r\nPacketLength = %d\r\n",dwLength));

	b = dwLength;
	while(b)
	{
		for(a=0; (a<16) && ((b-a) > 0);a++)
			DEBUGMSG(ZONE_INFO, (L"%.2x ",pbData[dwLength-b+a]));
		DEBUGMSG(ZONE_INFO, (L"\t"));
		for(a=0; (a<16) && ((b-a) > 0);a++)
			if(IS_ALPHANUM(pbData[dwLength-b+a]))
				DEBUGMSG(ZONE_INFO, (L"%c",pbData[dwLength-b+a]));
			else
				DEBUGMSG(ZONE_INFO, (L"."));
		DEBUGMSG(ZONE_INFO, (L"\r\n"));
		b -= a;
	}
	DEBUGMSG(ZONE_INFO, (L"\r\n"));
}

void LPC_PrintTSV(P_ETHERNET_REGS_T pEMAC)
{
	DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"pEMAC->TSV0      = 0x%.8x\t(",pEMAC->tsv0));
	if((pEMAC->tsv0 & TSV0_CRC_ERROR) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"CRCError "));
	if((pEMAC->tsv0 & TSV0_LEN_CHECK_ERROR) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"LengthCheck "));
	if((pEMAC->tsv0 & TSV0_LEN_OUT_OF_RANGE) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"OutOfRange "));
	if((pEMAC->tsv0 & TSV0_DONE) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"DONE "));
	if((pEMAC->tsv0 & TSV0_MULTICAST) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"MULTI "));
	if((pEMAC->tsv0 & TSV0_BROADCAST) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"BROAD "));
	if((pEMAC->tsv0 & TSV0_PACKET_DEFER) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"DEFERED "));
	if((pEMAC->tsv0 & TSV0_EXCESSIVE_DEFER) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"EXCESS_DEFER "));
	if((pEMAC->tsv0 & TSV0_EXCESSIVE_COLLISION) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"COLL "));
	if((pEMAC->tsv0 & TSV0_LATE_COLLISION) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"LATE_COLL "));
	if((pEMAC->tsv0 & TSV0_GIANT) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"GIANT "));
	if((pEMAC->tsv0 & TSV0_UNDERRUN) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"UNDER "));
	
	DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"%d bytes ",(pEMAC->TSV0 & TSV0_TOTAL_BYTES_MASK) >> TSV0_TOTAL_BYTES_OFFSET));
	
	if((pEMAC->tsv0 & TSV0_VLAN) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"VLAN "));
	if((pEMAC->tsv0 & TSV0_BACKPRESSURE) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"BACK "));
	if((pEMAC->tsv0 & TSV0_PAUSE) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"PAUSE "));
	if((pEMAC->tsv0 & TSV0_CONTROLFRAME) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"CTRL "));

	DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L")\r\n"));

	DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"pEMAC->TSV1      = 0x%.8x\t(%d bytes ; %d coll)\r\n",
					pEMAC->TSV1,
					(pEMAC->TSV1 & TSV1_TXBYTECOUNT_MASK) >> TSV1_TXBYTECOUNT_OFFSET,
					(pEMAC->TSV1 & TSV1_TXCOLCOUNT_MASK) >> TSV1_TXCOLCOUNT_OFFSET));
}
void LPC_PrintRSV(P_ETHERNET_REGS_T pEMAC)
{
	DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"pEMAC->RSV       = 0x%.8x\t(",pEMAC->RSV));
	DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"%d bytes ",(pEMAC->RSV & RSV_RCV_BYTES_CNT_MASK) >> RSV_RCV_BYTES_CNT_OFFSET));
	if((pEMAC->rsv & RSV_PCKT_PRV_IGNORED) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"PCKT_PREV_IGN "));
	if((pEMAC->rsv & RSV_RXDV_EVT_PRV_SEEN) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"RSV_RXDV_EVT_PRV_SEEN "));
	if((pEMAC->rsv & RSV_CARRIER_EVT_PRV_SEEN) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"RSV_CARRIER_EVT_PRV_SEEN "));
	if((pEMAC->rsv & RSV_RECEIVED_CODE_VIOLATION) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"RX_CODE_VIOL "));
	if((pEMAC->rsv & RSV_CRC_ERROR) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"CRC_ERR "));
	if((pEMAC->rsv & RSV_LEN_CHECK_ERROR) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"LEN_CHK_ERR "));
	if((pEMAC->rsv & RSV_LEN_OUT_OF_RANGE) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"LEN_OOR "));
	if((pEMAC->rsv & RSV_RECEIVE_OK) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"RECEIVE_OK "));
	if((pEMAC->rsv & RSV_MULTICAST) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"MULTI "));
	if((pEMAC->rsv & RSV_BROADCAST) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"BROAD "));
	if((pEMAC->rsv & RSV_DRIBBLE_NIBBLE) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"DRBL_NIBL "));
	if((pEMAC->rsv & RSV_CTRL_FRAME) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"CTRL_FRM "));
	if((pEMAC->rsv & RSV_PAUSE) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"PAUSE "));
	if((pEMAC->rsv & RSV_UNSUPPORTED_OP) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"UNSUPPORTED "));
	if((pEMAC->rsv & RSV_VLAN) != 0)
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"VLAN "));
	DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L")\r\n"));
}


/*
Function Name : LAN91CNDIS-MiniportHalt       
Description   :
MiniportHalt is a required function that de-allocates resources 
when the network adapter is removed and halts the network adapter
Parameters    :    
NDIS_HANDLE AdapterContext - Specifies the handle to a 
miniport-allocated context area 
Return Value  :
VOID
*/
VOID LPC3xxx_NDIS_MiniportHalt(NDIS_HANDLE AdapterContext)
{
	PMINIPORT_ADAPTER   pAdapter = (MINIPORT_ADAPTER *) AdapterContext;
	BackOut(pAdapter);
	return;
}


/*
Function Name : LPC3250_NDIS_MiniportQueryInformation
Description   : This function is a required function 
that returns information about the capabilities 
and status of the driver and/or its network adapter.
Parameters    :
NDIS_HANDLE AdapterContext - Handle to the adapter structure
NDIS_OID    Oid - OID code designation the query operation that the driver should carr out
PVOID       InformationBuffer - pointer to the buffer in which the driver return the value
ULONG       InformationBufferLength - lenght of the buffer
PULONG      BytesWritten - number of bytes the driver is returning
PULONG      BytesNeeded - additional bytes, if needed, to satisfy the query.
Return Value  :
NDIS_STATUS Status
*/
NDIS_STATUS LPC3xxx_NDIS_MiniportQueryInformation(	NDIS_HANDLE AdapterContext,
													NDIS_OID    Oid,
													PVOID       InformationBuffer,
													ULONG       InformationBufferLength,
													PULONG      BytesWritten,
													PULONG      BytesNeeded)
{
	MINIPORT_ADAPTER	*Adapter	= (MINIPORT_ADAPTER *) AdapterContext;
	NDIS_STATUS			Status		= NDIS_STATUS_SUCCESS;
	NDIS_OID			ReturnData;
    void				*Source		= &ReturnData;
    UINT				BytesToMove	= sizeof(NDIS_OID);
	
    
	DEBUGMSG(ZONE_INFO, (TEXT("LPC3250 NDIS ==> MiniportQuery Information OID=%x, "), Oid));
	switch(Oid)
    {
		case OID_GEN_SUPPORTED_LIST:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_SUPPORTED_LIST\r\n")));
			Source      = (void *) &GlobalObjects;
			BytesToMove = sizeof(GlobalObjects);
			break;

		case OID_GEN_MEDIA_SUPPORTED:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_MEDIA_SUPPORTED\r\n")));
			ReturnData = NdisMedium802_3;
			break;

		case OID_GEN_MEDIA_IN_USE:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_MEDIA_IN_USE\r\n")));
			ReturnData = NdisMedium802_3;
			break;

		case OID_GEN_MAXIMUM_LOOKAHEAD:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_MAXIMUM_LOOKAHEAD\r\n")));
			ReturnData = Adapter->dwLookAheadBufferSize;
			break;
			
		case OID_GEN_MAXIMUM_FRAME_SIZE:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_MAXIMUM_FRAME_SIZE\r\n")));
			ReturnData = MAX_FRAME_SIZE;
			break;		

		case OID_GEN_LINK_SPEED:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_LINK_SPEED\r\n")));
			ReturnData = Adapter->b100Mbps ? 100000 : 10000;
			break;		

		case OID_GEN_TRANSMIT_BUFFER_SPACE: 
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_TRANSMIT_BUFFER_SPACE\r\n")));
			ReturnData = MAX_FRAME_SIZE;
			break;

		case OID_GEN_RECEIVE_BUFFER_SPACE: 
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_RECEIVE_BUFFER_SPACE\r\n")));
			ReturnData = Adapter->dwRxStrides * MAX_FRAME_SIZE;		
			break;

		case OID_GEN_TRANSMIT_BLOCK_SIZE: 
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_TRANSMIT_BLOCK_SIZE\r\n")));
			ReturnData = MAX_FRAME_SIZE;
			break;

		case OID_GEN_RECEIVE_BLOCK_SIZE: 
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_RECEIVE_BLOCK_SIZE\r\n")));
			ReturnData = MAX_FRAME_SIZE;
			break;

		case OID_GEN_VENDOR_ID:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_VENDOR_ID\r\n")));
			ReturnData = 0;
			NdisMoveMemory((void *) &ReturnData,(void *) &Adapter->MACAddress,	3);
			break;

		case OID_GEN_VENDOR_DESCRIPTION: 
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_VENDOR_DESCRIPTION\r\n")));
			Source      = (void *) DRV_VENDOR_NAME;
			BytesToMove = SIZE_DRV_VENDOR_NAME;
			break;

		case OID_GEN_CURRENT_LOOKAHEAD:  
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_CURRENT_LOOKAHEAD\r\n")));
			ReturnData = MAX_FRAME_SIZE;
			break;

		case OID_GEN_DRIVER_VERSION:  
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_DRIVER_VERSION\r\n")));
			BytesToMove = 2;
			ReturnData  = DRIVER_NDIS_VERSION;
			break;

		case OID_GEN_MAXIMUM_TOTAL_SIZE:  
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_MAXIMUM_TOTAL_SIZE\r\n")));
			ReturnData = MAX_FRAME_SIZE;
			break;

		case OID_GEN_MAC_OPTIONS: 
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_MAC_OPTIONS\r\n")));
			ReturnData =	NDIS_MAC_OPTION_COPY_LOOKAHEAD_DATA |
							NDIS_MAC_OPTION_RECEIVE_SERIALIZED  |
							NDIS_MAC_OPTION_TRANSFERS_NOT_PEND  |
							NDIS_MAC_OPTION_NO_LOOPBACK;
			break;

		case OID_802_3_PERMANENT_ADDRESS:
			DEBUGMSG(ZONE_INIT, (TEXT("OID_802_3_PERMANENT_ADDRESS\r\n")));
			Source      = (void *) &Adapter->MACAddress;
			BytesToMove = MAC_ADDRESS_SIZE;
			break;

		case OID_802_3_CURRENT_ADDRESS: 
			DEBUGMSG(ZONE_INFO, (TEXT("OID_802_3_CURRENT_ADDRESS\r\n")));
			Source      = (void *) &Adapter->MACAddress;
			BytesToMove = MAC_ADDRESS_SIZE;
			break;

		case OID_802_3_MULTICAST_LIST:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_802_3_MULTICAST_LIST\r\n")));
			BytesToMove = Adapter->tMulticastTable.MulticastTableEntryCount 
				* ETH_LENGTH_OF_ADDRESS;
			Source = (void *) Adapter->tMulticastTable.MulticastTableEntry;
			break;

		case OID_802_3_MAXIMUM_LIST_SIZE: 
			DEBUGMSG(ZONE_INFO, (TEXT("OID_802_3_MAXIMUM_LIST_SIZE\r\n")));
			ReturnData = MAX_MULTICAST_ADDRESS;
			break;

/*
		case OID_802_3_RCV_ERROR_ALIGNMENT:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_802_3_RCV_ERROR_ALIGNMENT\r\n")));
			ReturnData = Adapter->Stat_AlignError;		
			break;

		case OID_802_3_XMIT_ONE_COLLISION:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_802_3_XMIT_ONE_COLLISION\r\n")));
			ReturnData = Adapter->Stat_SingleColl;
			break;		
	        
		case OID_802_3_XMIT_MORE_COLLISIONS:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_802_3_XMIT_MORE_COLLISIONS\r\n")));
			ReturnData = Adapter->Stat_MultiColl;
			break;
*/
		case OID_GEN_MEDIA_CONNECT_STATUS:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_MEDIA_CONNECT_STATUS\r\n")));
			LPC3xxx_NDIS_MiniportUpdateLink(Adapter);
			if(!Adapter->bMediaConnected)
			{
				ReturnData = NdisMediaStateDisconnected;
				DEBUGMSG(ZONE_INFO, (L"NdisMediaStateDisconnected\r\n"));
			}
			else
			{
				ReturnData = NdisMediaStateConnected;
				DEBUGMSG(ZONE_INFO, (L"NdisMediaStateConnected\r\n"));
			}
			break;

		case OID_GEN_VENDOR_DRIVER_VERSION:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_VENDOR_DRIVER_VERSION\r\n")));
			ReturnData = ((DRIVER_NDIS_MINOR_VERSION >> 16 ) | 
				DRIVER_NDIS_MAJOR_VERSION);
			break;

		case OID_GEN_MAXIMUM_SEND_PACKETS:
			DEBUGMSG(ZONE_INFO, (TEXT("OID_GEN_MAXIMUM_SEND_PACKETS\r\n")));
			ReturnData = 1;
			break;
/*
		case OID_GEN_XMIT_OK:
			ReturnData = Adapter->Stat_TxOK;
			break;		
			
		case OID_GEN_RCV_OK:
			ReturnData = Adapter->Stat_RxOK;
			break;
			
		case OID_GEN_XMIT_ERROR:
			ReturnData = Adapter->Stat_TxError;
			break;
			
		case OID_GEN_RCV_ERROR:
			ReturnData = Adapter->Stat_RxError;
			break;
			
		case OID_GEN_RCV_NO_BUFFER:
			ReturnData = Adapter->Stat_RxOvrn;
			break;
*/
		case OID_GEN_HARDWARE_STATUS:
			if (Adapter->eState == NORMAL_STATE)
				ReturnData = NdisHardwareStatusReady;
			else
			{
				if(Adapter->eState == INITIALIZING_STATE)
					ReturnData = NdisHardwareStatusNotReady;
				if(Adapter->eState == RESET_STATE)
					ReturnData = NdisHardwareStatusReset;
			}
			break;

		default:
			DEBUGMSG(ZONE_INFO, (TEXT("Unknown or Unsupported OID Statistics Query\r\n")));
			BytesToMove = 0;
			Status      = NDIS_STATUS_NOT_SUPPORTED;
			break;
	}
	
	
	if(BytesToMove)
    {
        Status = CopyInfo(	InformationBuffer,
							Source,
							InformationBufferLength,
							BytesToMove,
							BytesWritten,
							BytesNeeded);
    }
	
	
	DEBUGMSG(ZONE_INFO, (TEXT("LPC3250 NDIS <== MiniportQuery Information OID\r\n")));
	return Status;
}

VOID	LPC3xxx_NDIS_MiniportUpdateLink(PMINIPORT_ADAPTER	pAdapter)
{/*
	int i;

	DEBUGMSG(ZONE_INFO, (L"LPC3xxx_NDIS_MiniportUpdateLink: Checking Link:\r\n"));
	//	Check the link status.
	for (i = 0; i < 0x10000; i++)
	{
		if(PHY_CheckLinkOK(pAdapter->pEMACRegs))
		{
			//	Link is on.
			break;
		}
	}

	if(i == 0x10000)
	{
		DEBUGMSG(ZONE_INFO | ZONE_ERROR, (L"LPC3xxx_NDIS_MiniportUpdateLink: Link Establishment Timed out\r\n"));
		pAdapter->bMediaConnected = FALSE;
	}
	else
	{
		pAdapter->bMediaConnected = TRUE;
	}

	DEBUGMSG(ZONE_INFO, (L"LINK OK\r\n"));*/
	pAdapter->bMediaConnected = PHY_CheckLinkOK(pAdapter->pEMACRegs);
}


NDIS_STATUS     LPC3xxx_NDIS_MiniportSend(NDIS_HANDLE  AdapterContext,
										  PNDIS_PACKET pNDISPacket,
										  UINT         Flags)
{
	PMINIPORT_ADAPTER	pAdapter = (PMINIPORT_ADAPTER) AdapterContext;
	UINT				PacketLength;
	DWORD				dwTimeOut = 10;	//	We loop 10 times maximum before exiting
	BOOL				bSent = FALSE;
	LPCPS_ETH_TX_STATUS pPacketStatus;
	LPCPS_ETH_DESC		pPacketDesc;
	DWORD				dwCurrentTxSlot;

	DEBUGMSG(ZONE_TX, (TEXT("++LPC3xxx_NDIS_MiniportSend\r\n")));
	
	//Make sure that the packet size is in legal limits.
	NdisQueryPacket(pNDISPacket,NULL,NULL,NULL,(PUINT) &PacketLength);

	if ((PacketLength > MAX_FRAME_SIZE) || (PacketLength < MIN_FRAME_SIZE))
	{
		DEBUGMSG(ZONE_TX, (TEXT("LPC3250 NDIS: Invalid Packet Size\r\n")));
		DEBUGMSG(ZONE_TX, (TEXT("LPC3250 NDIS <== Miniport Send\r\n")));
		return (NDIS_STATUS_FAILURE);
	}

	//	We check if the EMAC is ready to send (aka we have a free descriptor)
	if(!EMAC_FrameTxRdy(pAdapter))	//	If we do not have enough resources, we signal it to the NDIS layer
	{
		DEBUGMSG(ZONE_TX, (TEXT("--LPC3xxx_NDIS_MiniportSend (Not enough Resources)\r\n")));
		return NDIS_STATUS_RESOURCES;
	}

	dwCurrentTxSlot = EMAC_TxSlotGetCurrentProducer(pAdapter);

	//	We fill up the status
	pPacketDesc = EMAC_TxDescGetCurrent(pAdapter);

	DEBUGMSG(ZONE_TX, (L"pPacketDesc                = 0x%.8x\r\n",pPacketDesc));
	DEBUGMSG(ZONE_TX, (L"pPacketDesc->PacketAddress = 0x%.8x\r\n",pPacketDesc->PacketAddress));
	DEBUGMSG(ZONE_TX, (L"pPacketDesc->Control       = 0x%.8x\r\n",pPacketDesc->Control));

	pPacketDesc->Control = ((PacketLength-1) & ETH_TX_DESC_SIZE) |
							ETH_TX_DESC_LAST |
							ETH_TX_DESC_INTERRUPT;

	pPacketStatus = EMAC_TxStatusGetCurrent(pAdapter);
	DEBUGMSG(ZONE_TX, (L"pPacketStatus              = 0x%.8x\r\n",pPacketStatus));
	DEBUGMSG(ZONE_TX, (L"pPacketStatus->StatusInfo  = 0x%.8x\r\n",pPacketStatus->StatusInfo));

	//	We clear all status to make sure
	pPacketStatus->StatusInfo = 0;

	//	We copy the frame into the buffer
	EMAC_TxBuffCopyCurrent(pAdapter, pNDISPacket);

	//	We produce and let the interrupt handle deals with it
	//		First we fill the structure to be used in the IT
	pAdapter->pTXPackets[dwCurrentTxSlot].pNDISPacket = pNDISPacket;
	pAdapter->pTXPackets[dwCurrentTxSlot].dwPacketSize = PacketLength;
	//		Then we produce it
	EMAC_TxDescProduce(pAdapter);

	DEBUGMSG(ZONE_TX, (TEXT("--LPC3xxx_NDIS_MiniportSend\r\n")));

	return (NDIS_STATUS_PENDING);	//	If we do not check and wait for completion here : RGUNS TODO
}

BOOL HardReset(PMINIPORT_ADAPTER pAdapter)
{
	BOOL rc = FALSE;
	DEBUGMSG(ZONE_INFO, (TEXT("++HardReset\r\n")));

	pAdapter->eState = RESET_STATE;

	//	Reseting PHY
	if(!PHY_SWReset(pAdapter->pEMACRegs))
	{
		DEBUGMSG(ZONE_INFO, (L"HardReset: PHY Reset failed.\r\n"));
		DEBUGMSG(ZONE_INFO, (TEXT("--HardReset\r\n")));
		return FALSE;
	}

	rc = AdapterReset(pAdapter);

	DEBUGMSG(ZONE_INFO, (TEXT("--HardReset(rc=%s)\r\n"),rc ? L"TRUE" : L"FALSE"));

	return rc;
}
BOOL AdapterReset(PMINIPORT_ADAPTER pAdapter)
{
	DWORD	i;
	DWORD	dwHClk,bytesret;
	DWORD	dwTempReg;

	DEBUGMSG(ZONE_INFO, (TEXT("++AdapterReset\r\n")));

	pAdapter->eState = RESET_STATE;

	//	Now the controller is on, we can access the registers of the MAC

	pAdapter->pEMACRegs->mcfg = MCFG_HCLK_DIV_28;	//	Only 1 PHY, no Suppressed Preamble, Host Clock divided by 28

	//  Reset all EMAC internal modules
	pAdapter->pEMACRegs->mac1 = MAC1_RESET_TX |
								MAC1_RESET_MCS_TX |
								MAC1_RESET_RX |
								MAC1_RESET_MCS_RX |
								MAC1_SIMULATION_RESET |
								MAC1_SOFT_RESET;

	pAdapter->pEMACRegs->mac1 = 0;

	pAdapter->pEMACRegs->command =	COMMAND_RXRESET |
									COMMAND_TXRESET	|
									COMMAND_REGRESET;

	//	A short delay after reset
	Sleep(100);

	pAdapter->pEMACRegs->command = 0;

	DEBUGMSG(ZONE_INFO, (L"AdapterReset: Initializing MAC.\r\n"));

	//	Initialize MAC control registers
	pAdapter->pEMACRegs->mac1 = MAC1_PASS_ALL_RCV;
	pAdapter->pEMACRegs->mac2 = MAC2_CRC_EN | MAC2_PAD_CRC_EN;
	pAdapter->pEMACRegs->maxf = MAX_FRAME_SIZE & MAXF_MAXIMUM_FRAME_LEN;
	pAdapter->pEMACRegs->clrt = CLRT_DEF;
	pAdapter->pEMACRegs->ipgr = IPGR_DEF;

	//	Select RMII or MII interface
		//	Enable Reduced MII interface
	pAdapter->pEMACRegs->command =	COMMAND_RMII |
									COMMAND_PASSRUNTFRAME;

	DEBUGMSG(ZONE_INIT, (L"AdapterReset: PHY reset\r\n"));

	PHY_SetInterface(pAdapter->pEMACRegs, pAdapter->bRMII);

	if(!PHY_CheckCompatibility(pAdapter->pEMACRegs))
	{
		DEBUGMSG(ZONE_INFO, (L"NDIS Miniport: PHY is not compatible with the current BSP.\r\n"));
		return FALSE;
	}

	//	Set the Ethernet MAC Address registers
	pAdapter->pEMACRegs->sa[0] = pAdapter->MACAddress[1] | (pAdapter->MACAddress[0] << 8);
	pAdapter->pEMACRegs->sa[1] = pAdapter->MACAddress[3] | (pAdapter->MACAddress[2] << 8);
	pAdapter->pEMACRegs->sa[2] = pAdapter->MACAddress[5] | (pAdapter->MACAddress[4] << 8);

	//	Initialize Tx and Rx DMA Descriptors
	DEBUGMSG(ZONE_INIT, (L"AdapterReset: Initializing Descriptor engine\r\n"));
	EMAC_InitEngine(pAdapter);

	pAdapter->dwLastConsumed = 0;	//	Setting the LastConsumed to the current Consumed => none consumed just yet

	DEBUGMSG(ZONE_INIT, (L"AdapterReset: Checking Link: "));
	LPC3xxx_NDIS_MiniportUpdateLink(pAdapter);
	
	if(pAdapter->bMediaConnected)
	{
		DEBUGMSG(ZONE_INIT, (L"LINK Established\r\n"));
	}
	else
	{
		DEBUGMSG(ZONE_INIT, (L"LINK Not Established\r\n"));
	}

	if(pAdapter->bAutoNeg)
	{
		if(/*pAdapter->bMediaConnected && */!PHY_InitLink(pAdapter->pEMACRegs))
		{
			pAdapter->bAutoNeg = FALSE;	//	We disable the Autoneg
		}
		else
		{
			//	Configure Full/Half Duplex mode.
			if(PHY_FullDuplex(pAdapter->pEMACRegs))
			{
				//	Full duplex is enabled.
				pAdapter->bFullDuplex = TRUE;
			}
			else
			{
				//	Half duplex mode.
				pAdapter->bFullDuplex = FALSE;
			}

			//	Configure 100MBit/10MBit mode.
			if(PHY_Speed(pAdapter->pEMACRegs))
			{
				//	100MBit mode.
				pAdapter->b100Mbps = TRUE;
			}
			else
			{
				//	10MBit mode
				pAdapter->b100Mbps = FALSE;
			}
		}
	}

	//	Setting the PHY in the right duplex mode (usually this settings is going to be ignored in AutoNeg)
	PHY_SetDuplex(pAdapter->pEMACRegs,pAdapter->bFullDuplex);

	if(pAdapter->bFullDuplex)
	{
		//	Full duplex is enabled.
		DEBUGMSG(ZONE_INIT, (L"AdapterReset: Full Duplex\r\n"));
		
		pAdapter->pEMACRegs->mac2    |= MAC2_FULL_DUPLEX;
		pAdapter->pEMACRegs->command |= COMMAND_FULLDUPLEX;
		pAdapter->pEMACRegs->ipgt     = IPGT_FULL_DUP;
	}
	else
	{
		//	Half duplex mode.
		DEBUGMSG(ZONE_INIT, (L"AdapterReset: Half Duplex\r\n"));

		pAdapter->pEMACRegs->mac2	 &= ~MAC2_FULL_DUPLEX;
		pAdapter->pEMACRegs->command &= ~COMMAND_FULLDUPLEX;
		pAdapter->pEMACRegs->ipgt	  =  IPGT_HALF_DUP;
	}

//	dwHClk = clkpwr_get_base_clock_rate(CLKPWR_PERIPH_CLK);
	if (KernelIoControl(IOCTL_LPC32XX_GETHCLK, NULL, 0, &dwHClk,
		sizeof (dwHClk), &bytesret) == FALSE)
		{
		RETAILMSG(ZONE_ERROR, 
            (TEXT("ERROR: EthNet: Error getting Einternet base clock rate.\r\n")));
		dwHClk = 104000000;
		}

	//	Configure 100MBit/10MBit mode.
	if(pAdapter->b100Mbps)
	{
		//	100MBit mode.
		DEBUGMSG(ZONE_INIT, (L"AdapterReset: 100 Mbps\r\n"));
		pAdapter->pEMACRegs->supp |= SUPP_SPEED_100MBPS;

		if(pAdapter->bRMII)	//	Are we in RMII ?
			i = dwHClk / 50000000;	//	Yes => Get Divisor for 50 MHz	(We will actually get 26 at most since we can only divide by 4)
		else
			i = dwHClk / 25000000;	//	No => Get Divisor for 25 MHz
	}
	else
	{
		//	10MBit mode
		DEBUGMSG(ZONE_INIT, (L"AdapterReset: 10 Mbps\r\n"));
		pAdapter->pEMACRegs->supp &= ~SUPP_SPEED_100MBPS;

		if(pAdapter->bRMII)	//	Are we in RMII ?
			i = dwHClk / 50000000;	//	Yes => Get Divisor for 50 MHz	(We will actually get 26 at most since we can only divide by 4)
		else
			i = (dwHClk / 25000000) * 10;	//	No => Get Divisor for 2.5 MHz (or as close as possible)
	}

	//	Now, let's get the closest range
	pAdapter->pEMACRegs->mcfg &= MCFG_CLK_SEL_MSK;
	if(i > 20)
		pAdapter->pEMACRegs->mcfg |= MCFG_HCLK_DIV_28;
	else if(i > 14)
		pAdapter->pEMACRegs->mcfg |= MCFG_HCLK_DIV_20;
	else if(i > 10)
		pAdapter->pEMACRegs->mcfg |= MCFG_HCLK_DIV_14;
	else if(i >  8)
		pAdapter->pEMACRegs->mcfg |= MCFG_HCLK_DIV_10;
	else if(i >  6)
		pAdapter->pEMACRegs->mcfg |= MCFG_HCLK_DIV_8;
	else if(i >  4)
		pAdapter->pEMACRegs->mcfg |= MCFG_HCLK_DIV_6;
	else
		pAdapter->pEMACRegs->mcfg |= MCFG_HCLK_DIV_4;

	dwTempReg = 0;

	if(pAdapter->bPerfectMatch)
	{
		dwTempReg |= RXFILTERCTRL_ACPT_PERFECT_EN;
		DEBUGMSG(1, (L"AdapterReset: Perfect Match enabled"));
	}
	if(pAdapter->bRxBroadcast)
	{
		dwTempReg |= RXFILTERCTRL_ACPT_BROADCAST_EN;
		DEBUGMSG(1, (L"AdapterReset: Broadcast enabled"));
	}
	if(pAdapter->bRxUnicastAll)
	{
		dwTempReg |= RXFILTERCTRL_ACPT_UNICAST_EN;
		DEBUGMSG(1, (L"AdapterReset: All Unicast enabled"));
	}
	if(pAdapter->bRxUnicastHash)
	{
		dwTempReg |= RXFILTERCTRL_ACPT_UNIHASH_EN;
		DEBUGMSG(1, (L"AdapterReset: Unicast Hash enabled"));
	}
	if(pAdapter->bRxMulticastHash)
	{
		dwTempReg |= RXFILTERCTRL_ACPT_MULTI_HASH_EN;
		DEBUGMSG(1, (L"AdapterReset: Multicast Hash enabled"));
	}
	if(pAdapter->bRxMulticastAll)
	{
		dwTempReg |= RXFILTERCTRL_ACPT_MULTICAST_EN;
		DEBUGMSG(1, (L"AdapterReset: All Multicast enabled"));
	}

	pAdapter->pEMACRegs->rxfliterctrl = dwTempReg;	//	Disable all filters (do not accept anything)

	if(pAdapter->bPromiscuous)
	{
		pAdapter->pEMACRegs->command |=  COMMAND_PASSRXFILTER;
		DEBUGMSG(1, (L"AdapterReset: Promiscuous enabled"));
	}
	else
	{
		pAdapter->pEMACRegs->command &= ~COMMAND_PASSRXFILTER;
		DEBUGMSG(1, (L"AdapterReset: Promiscuous disabled"));
	}

	//	Reset all interrupts
	pAdapter->pEMACRegs->intclear  = 0xFFFF;

	//	Enable EMAC interrupts.
	DEBUGMSG(ZONE_INIT, (L"AdapterReset: Enabling Interrupts\r\n"));
	pAdapter->pEMACRegs->intenable =	INT_RX_INT |
										INT_TX_INT;

	//	Enable receive and transmit mode of MAC Ethernet core
	DEBUGMSG(ZONE_INIT, (L"AdapterReset: Enabling RX & TX Paths\r\n"));
	pAdapter->pEMACRegs->command  |= (	COMMAND_RX_EN	|
										COMMAND_TX_EN	);

	pAdapter->pEMACRegs->mac1     |= MAC1_RECEIVE_EN;
/*
	if(pAdapter->bMediaConnected)
		pAdapter->eState = LINKDOWN_STATE;
	else*/
		pAdapter->eState = NORMAL_STATE;

	DEBUGMSG(ZONE_INFO, (TEXT("--AdapterReset\r\n")));

	return TRUE;
}

/*
Function Name : LPC3xxx_NDIS_MiniportReset
Description   : This function is a required function that issues a hardware reset 
to the network adapter and/or resets the driver’s software state.
Parameters    : 
PBOOLEAN    AddressingReset -Points to a variable that MiniportReset
sets to TRUE if the NDIS library should call 
MiniportSetInformation to restore addressing information 
to the current values. 
NDIS_HANDLE AdapterContext - Handle to the adapter structure

  Return Value  :
  NDIS_STATUS Status
*/
NDIS_STATUS	LPC3xxx_NDIS_MiniportReset(	PBOOLEAN    AddressingReset,
										NDIS_HANDLE AdapterContext)
{
	NDIS_STATUS			Status		= NDIS_STATUS_SUCCESS;
	PMINIPORT_ADAPTER	pAdapter	= (PMINIPORT_ADAPTER) AdapterContext;

	DEBUGMSG(ZONE_INIT, (TEXT("LPC3250 NDIS ==> Miniport Reset\r\n")));

	AdapterReset(pAdapter);

	DEBUGMSG(ZONE_INIT, (TEXT("LPC3250 NDIS <== Miniport Reset\r\n")));
	return Status;
}

/*
Function Name : LPC3xxx_NDIS_MiniportCheckforHang
Description   : This function checks for the internal state of the chip. 
Return Value  : TRUE - If the chip needs to be reset, else FALSE
*/
BOOLEAN LPC3xxx_NDIS_MiniportCheckforHang(NDIS_HANDLE AdapterContext)
{
	BOOL bPreviousLinkState;
	PMINIPORT_ADAPTER	pAdapter = (PMINIPORT_ADAPTER) AdapterContext;

	DEBUGMSG(ZONE_INFO, (L"LPC3xxx_NDIS_MiniportCheckforHang: Checking Link: "));

	bPreviousLinkState = pAdapter->bMediaConnected;	//	We save the previous state

	LPC3xxx_NDIS_MiniportUpdateLink(pAdapter);
	
	if(bPreviousLinkState != pAdapter->bMediaConnected)
	{
		if(pAdapter->bMediaConnected)
		{
			DEBUGMSG(ZONE_INFO, (L"LINK Established\r\n"));
		}
		else
		{
			DEBUGMSG(ZONE_INFO, (L"LINK Not Established\r\n"));
		}
	}
	if(pAdapter->bMediaConnected)
		pAdapter->eState = LINKDOWN_STATE;

	return	(!pAdapter->bMediaConnected) || (bPreviousLinkState != pAdapter->bMediaConnected);//	bMediaConnected is equal to TRUE if we are connected, thus we invert it
	//return FALSE;
}
