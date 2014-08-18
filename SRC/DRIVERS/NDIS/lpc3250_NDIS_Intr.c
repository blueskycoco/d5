
#include <Ndis.h>
#include <PKFuncs.h>
#include "LPC3250_NDIS_Adapter.h"
#include "LPC3250_NDIS_Proto.h"

//	Local functions prototypes
void LPC_RXIntHandler(PMINIPORT_ADAPTER pAdapter);
void LPC_TXIntHandler(PMINIPORT_ADAPTER pAdapter);
void LPC_RXErrorIntHandler(PMINIPORT_ADAPTER pAdapter);
void LPC_RXFatalErrorIntHandler(PMINIPORT_ADAPTER pAdapter);
void LPC_TXFatalErrorIntHandler(PMINIPORT_ADAPTER pAdapter);

/*
Function Name : 	LPC3xxx_NDIS_MiniportISR
Description   :
This routine is invoked when an interrupt occurs.  If
the interrupting device is identified as ours, then the DPC
is scheduled to complete the processing.
Parameters    :
OUT PBOOLEAN     InterruptRecognized            Is it ours?
OUT PBOOLEAN     QueueMiniportHandleInterrupt   Run the DPC?
IN  NDIS_HANDLE  AdapterContext                 The Adapter structure.

  Return Value  :
  VOID
*/
VOID LPC3xxx_NDIS_MiniportISR(	PBOOLEAN	InterruptRecognized,
								PBOOLEAN	QueueMiniportHandleInterrupt,
								NDIS_HANDLE	MiniportAdapterContext)
{
	DEBUGMSG(ZONE_INTR, (TEXT("++LPC3xxx_NDIS_MiniportISR\r\n")));

	//	RGUNS: no need to mask the interrupts since the IRQ line is dedicated to the EMAC, so should be mask at OAL level
	LPC3xxx_NDIS_MiniportDisableInterrupt(MiniportAdapterContext);	//	Yes since InterruptDone is called from NDIS layer as soon as we return from here !

	*InterruptRecognized = TRUE;			//	We did recognize the interrupt
	*QueueMiniportHandleInterrupt = TRUE;	//	We need more processing through the MiniportHandleInterrupt function

	DEBUGMSG(ZONE_INTR, (TEXT("--LPC3xxx_NDIS_MiniportISR\r\n")));
}

/*
Function Name : 	LPC3xxx_NDIS_MiniPortHandleInterrupt
Description   :    
This routine performs deferred processing for adapter
interrupts.  All pending interrupt conditions are
handled before exit.
Parameters    :
NDIS_HANDLE AdapterContext - Handle to the adapter structure

  Return Value  :
  VOID
*/
VOID LPC3xxx_NDIS_MiniPortHandleInterrupt(IN NDIS_HANDLE  AdapterContext)
{
	PMINIPORT_ADAPTER	pAdapter = (PMINIPORT_ADAPTER) AdapterContext;
	DWORD				dwIntStatus;

	DEBUGMSG(ZONE_INTR, (TEXT("++LPC3xxx_NDIS_MiniPortHandleInterrupt\r\n")));

	while(1)
	{
		dwIntStatus = pAdapter->pEMACRegs->intstatus;

		if(dwIntStatus == 0)	//	Do we have any interrupt?
		{
			break;
		}

		//	We treat the fatal errors
		if((dwIntStatus & INT_RXOVERRUN) != 0)	//	Fatal Overrun => need RxReset
		{
			LPC_RXFatalErrorIntHandler(pAdapter);
		}
		if((dwIntStatus & INT_TXUNDERRUN) != 0)	//	Fatal Underrun => need TxReset
		{
			LPC_TXFatalErrorIntHandler(pAdapter);
		}

		//	We treat the received/send queue
		if((dwIntStatus & INT_RX_INT) != 0)	//	We have a receiving queue
		{
			LPC_RXIntHandler(pAdapter);
		}

		if((dwIntStatus & INT_TX_INT) != 0)	//	We finished to send (a) packet(s)
		{
			LPC_TXIntHandler(pAdapter);
		}
	}

	LPC3xxx_NDIS_MiniportEnableInterrupt(pAdapter);

	DEBUGMSG(ZONE_INTR, (TEXT("--LPC3xxx_NDIS_MiniPortHandleInterrupt\r\n")));
}

VOID LPC3xxx_NDIS_MiniportEnableInterrupt(NDIS_HANDLE MiniportAdapterContext)
{
	PMINIPORT_ADAPTER    pAdapter = (PMINIPORT_ADAPTER) MiniportAdapterContext;

	DEBUGMSG(ZONE_INTR, (TEXT("++LPC3xxx_NDIS_MiniportEnableInterrupt\r\n")));

	//	Enable Receive and Transmit interrupts
	pAdapter->pEMACRegs->intenable =	INT_RX_INT |
										INT_TX_INT;

	DEBUGMSG(ZONE_INTR, (TEXT("--LPC3xxx_NDIS_MiniportEnableInterrupt\r\n")));
}

VOID LPC3xxx_NDIS_MiniportDisableInterrupt(NDIS_HANDLE MiniportAdapterContext)
{
	PMINIPORT_ADAPTER    pAdapter = (PMINIPORT_ADAPTER) MiniportAdapterContext;
	
	DEBUGMSG(ZONE_INTR, (TEXT("++LPC3xxx_NDIS_MiniportDisableInterrupt\r\n")));

	//	Disable all interrupts
	pAdapter->pEMACRegs->intenable = 0;

	DEBUGMSG(ZONE_INTR, (TEXT("--LPC3xxx_NDIS_MiniportDisableInterrupt\r\n")));
}

void LPC_RXIntHandler(PMINIPORT_ADAPTER pAdapter)
{
	LPCPS_ETH_RX_STATUS pCurrentStatus;
	DWORD				dwErrorStatus;
	DWORD				dwPacketSize;
	BOOL				bReceivedFrame;

	DEBUGMSG(ZONE_RX, (TEXT("++LPC_RXIntHandler\r\n")));

	bReceivedFrame = FALSE;	//	We assume we won't get a complete frame just yet

	//	Did we receive a frame?
	if(EMAC_FrameRxd(pAdapter))
	{
		//	Yes
		//	Then we retrieve the current descriptor to treat
		pCurrentStatus = EMAC_RxStatusGetCurrent(pAdapter);

		dwErrorStatus = pCurrentStatus->StatusInfo & (	//ETH_RX_STATUS_INFO_ERROR |
														ETH_RX_STATUS_INFO_NODESC |
														ETH_RX_STATUS_INFO_OVERRUN |
														ETH_RX_STATUS_INFO_ALIGN_ERR |
														ETH_RX_STATUS_INFO_FAILFILTER);

		//	Now, we need to check if it is an error free frame
		if(dwErrorStatus != 0)	//	We discard the Range Fake Error
		{
			LPC_PrintRSV(pAdapter->pEMACRegs);
			//	An error occured
			DEBUGMSG(ZONE_RX, (L"LPC_RXIntHandler: pCurrentStatus->StatusInfo: 0x%.8x\r\n", pCurrentStatus->StatusInfo));

			//	Lets discard the frame
			EMAC_RxDiscardFrame(pAdapter);

			goto clean_up;
		}

		dwPacketSize = (pCurrentStatus->StatusInfo & ETH_RX_STATUS_INFO_RX_SIZE_MSK);

		if(dwPacketSize == 0)
		{
			DEBUGMSG(ZONE_RX, (L"LPC_RXIntHandler: ERROR: Frame has zero size.\r\n"));

			LPC_PrintRSV(pAdapter->pEMACRegs);

			//	Lets discard the frame
			EMAC_RxDiscardFrame(pAdapter);

			goto clean_up;
		}

		//	Can we fit that frame into the buffer ?
		if(dwPacketSize > pAdapter->dwLookAheadBufferSize)
		{
			//	No
			DEBUGMSG(ZONE_RX, (L"LPC_EthGetFrame: ERROR: too long for buffer (buffer is %d bytes, needs %d bytes).\r\n", pAdapter->dwLookAheadBufferSize, dwPacketSize));

			LPC_PrintRSV(pAdapter->pEMACRegs);
			//	Lets discard the frame
			EMAC_RxDiscardFrame(pAdapter);

			goto clean_up;
		}

		//	Do we need to do this here or in the error management fct?
		if((pAdapter->pEMACRegs->intstatus & (INT_RXERROR | INT_RXOVERRUN)) != 0)
		{
			DEBUGMSG(ZONE_RX, (L"pEMACRegs->IntStatus = 0x%.8x\r\n",pAdapter->pEMACRegs->IntStatus));
			LPC_PrintRSV(pAdapter->pEMACRegs);
			if((pAdapter->pEMACRegs->rsv & (RSV_LEN_CHECK_ERROR | RSV_CRC_ERROR | RSV_RECEIVED_CODE_VIOLATION)) != 0)
			{
				EMAC_RxDiscardFrame(pAdapter);

				goto clean_up;
			}
		}

		//	If we are here, the frame is all good :)

		//	We copy it into the LookAhead buffer
		EMAC_RxBuffCopyCurrent(pAdapter,pAdapter->pVALookAheadBuffer,(WORD)dwPacketSize);

		if((pCurrentStatus->StatusInfo & ETH_RX_STATUS_INFO_LASTFLAG) != 0)	//	If it is the last frame, signal it
		{
			bReceivedFrame = TRUE;
			DEBUGMSG(ZONE_RX, (L"Was last frame of the packet\r\n"));
		}

		//	We update the consume index to release the buffer (it releases the DMA buffer)
		EMAC_RxDescConsume(pAdapter);

		//	We make sure that we'll signal the NDIS upper later of availability
		NdisMEthIndicateReceive(pAdapter->hAdapter,
								NULL,		//	No need for it since no MiniportTransferData function
								(void *) pAdapter->pVALookAheadBuffer,	//	Header
								(UINT) ETHERNET_HEADER_SIZE,			//	Header size
								(void *) (pAdapter->pVALookAheadBuffer + ETHERNET_HEADER_SIZE),	//	Actual data
								dwPacketSize - ETHERNET_HEADER_SIZE,
								dwPacketSize - ETHERNET_HEADER_SIZE);

//		DEBUGMSG(ZONE_RX, (L"Header:\r\n"));
//		LPC_PrintFrame(pAdapter->pVALookAheadBuffer,ETHERNET_HEADER_SIZE);
//		DEBUGMSG(ZONE_RX, (L"Body:\r\n"));
//		LPC_PrintFrame((void *) (pAdapter->pVALookAheadBuffer + ETHERNET_HEADER_SIZE),dwPacketSize - ETHERNET_HEADER_SIZE);
	}

clean_up:

	if((pAdapter->pEMACRegs->intstatus & INT_RXERROR) != 0)
		LPC_RXErrorIntHandler(pAdapter);

	if(bReceivedFrame == TRUE)
	{
		NdisMEthIndicateReceiveComplete(pAdapter->hAdapter);
	}

	if(!EMAC_FrameRxd(pAdapter))	//	If we do not have anymore packet to treat in reception
		pAdapter->pEMACRegs->intclear = INT_RXFINISHED | INT_RXDONE;

	DEBUGMSG(ZONE_RX, (TEXT("--LPC_RXIntHandler\r\n")));
}

void LPC_TXIntHandler(PMINIPORT_ADAPTER pAdapter)
{
	PNDIS_PACKET	pPacket;
	BOOL			bSent;
	LPCPS_ETH_TX_STATUS pPacketStatus;

	DEBUGMSG(ZONE_TX, (TEXT("++LPC_TXIntHandler\r\n")));

	while(pAdapter->dwLastConsumed != EMAC_TxSlotGetLastConsumed(pAdapter))
	{
		pPacketStatus = EMAC_TxStatusGetIndex(pAdapter, pAdapter->dwLastConsumed);
		pPacket = pAdapter->pTXPackets[pAdapter->dwLastConsumed].pNDISPacket;

		DEBUGMSG(ZONE_TX, (L"Sending Status:\r\n"));
		DEBUGMSG(ZONE_TX, (L"pPacketStatus->StatusInfo  = 0x%.8x\r\n",pPacketStatus->StatusInfo));
		DEBUGMSG(ZONE_TX, (L"g_pEthernet->TxProduceIndex= 0x%.8x\r\n",pAdapter->pEMACRegs->TxProduceIndex));
		DEBUGMSG(ZONE_TX, (L"g_pEthernet->TxConsumeIndex= 0x%.8x\r\n",pAdapter->pEMACRegs->TxConsumeIndex));
		DEBUGMSG(ZONE_TX, (L"pAdapter->dwLastConsumed   = %d\r\n",pAdapter->dwLastConsumed));
		if((pPacketStatus->StatusInfo & ETH_TX_STATUS_INFO_ERROR) != 0)
		{
			DEBUGMSG(ZONE_ERROR | ZONE_TX, (L"LPC_TXIntHandler: ERROR: Frame not sent\r\n"));
			bSent = FALSE;
		}
		else
		{
			bSent = TRUE;
		}

		if(bSent)
			NdisMSendComplete(pAdapter->hAdapter, pPacket, NDIS_STATUS_SUCCESS);
		else
			NdisMSendComplete(pAdapter->hAdapter, pPacket, NDIS_STATUS_FAILURE);

		pAdapter->dwLastConsumed = (pAdapter->dwLastConsumed + 1) % pAdapter->dwTxStrides;	//	We increment dwLastConsumed with wrap around
	}

	pAdapter->pEMACRegs->intclear = INT_TX_INT;	//	We Clear the interrupts even though they should not have happened
	DEBUGMSG(ZONE_TX, (TEXT("--LPC_TXIntHandler\r\n")));
}

void LPC_RXErrorIntHandler(PMINIPORT_ADAPTER pAdapter)
{
	DEBUGMSG(ZONE_ERROR, (TEXT("++LPC_RXErrorIntHandler\r\n")));

	DEBUGMSG(ZONE_ERROR, (L"pEMACRegs->IntStatus = 0x%.8x\r\n",pAdapter->pEMACRegs->IntStatus));
	LPC_PrintRSV(pAdapter->pEMACRegs);
	if((pAdapter->pEMACRegs->rsv & (RSV_LEN_CHECK_ERROR | RSV_CRC_ERROR | RSV_RECEIVED_CODE_VIOLATION)) != 0)
	{
		DEBUGMSG(ZONE_ERROR, (L"pEMACRegs->IntStatus = 0x%.8x\r\n",pAdapter->pEMACRegs->IntStatus));
		DEBUGMSG(ZONE_ERROR, (L"pEMACRegs->RSV      = 0x%.8x\r\n",pAdapter->pEMACRegs->RSV));
	}

	pAdapter->pEMACRegs->intclear = INT_RXERROR;

	DEBUGMSG(ZONE_ERROR, (TEXT("--LPC_RXErrorIntHandler\r\n")));
}

void LPC_RXFatalErrorIntHandler(PMINIPORT_ADAPTER pAdapter)
{
	//	Fatal Overrun => need RxReset
	DEBUGMSG(ZONE_ERROR, (TEXT("++LPC_RXFatalErrorIntHandler\r\n")));

	DEBUGMSG(ZONE_ERROR, (TEXT("Reseting Rx Datapath\r\n")));

	//	First we disable the RX path
	pAdapter->pEMACRegs->command &= ~COMMAND_RX_EN;

	//	Then we reset the RX path
	pAdapter->pEMACRegs->command |= COMMAND_RXRESET;
	pAdapter->pEMACRegs->command &= ~COMMAND_RXRESET;	//	Need to make sure we do not stay in a reset state

	//	Finally, we re-enable the Rx datapath
	pAdapter->pEMACRegs->command |= COMMAND_RX_EN;

	//	Finally we clear the interrupt bits
	pAdapter->pEMACRegs->intclear = INT_RX_INT;

	DEBUGMSG(ZONE_ERROR, (TEXT("--LPC_RXFatalErrorIntHandler\r\n")));
}

void LPC_TXFatalErrorIntHandler(PMINIPORT_ADAPTER pAdapter)
{
	DEBUGMSG(ZONE_ERROR, (TEXT("++LPC_TXFatalErrorIntHandler\r\n")));
	DEBUGMSG(ZONE_ERROR, (TEXT("Reseting Tx Datapath\r\n")));

	//	First we disable the RX path
	pAdapter->pEMACRegs->command &= ~COMMAND_TX_EN;

	//	Then we reset the TX path
	pAdapter->pEMACRegs->command |= COMMAND_TXRESET;
	pAdapter->pEMACRegs->command &= ~COMMAND_TXRESET;	//	Need to make sure we do not stay in a reset state

	//	Finally, we re-enable the Tx datapath
	pAdapter->pEMACRegs->command |= COMMAND_TX_EN;

	//	Finally we clear the interrupt bits
	pAdapter->pEMACRegs->intclear = INT_TX_INT;

	DEBUGMSG(ZONE_ERROR, (TEXT("--LPC_TXFatalErrorIntHandler\r\n")));
}