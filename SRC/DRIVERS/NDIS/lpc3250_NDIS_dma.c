#include <Ndis.h>
#include "LPC3250_NDIS_Adapter.h"
#include "LPC3250_NDIS_Proto.h"

/* RX and TX buffer definitions. */
#define RX_BUF(BASE, i)           (BASE + MAX_FRAME_SIZE*i)
#define TX_BUF(BASE, i)           (BASE + MAX_FRAME_SIZE*i)


//	function initializes Rx Descriptors
static void rx_descr_init (PMINIPORT_ADAPTER pAdapter)
{
	DWORD			i;
	LPCPS_ETH_DESC	pRxDescriptors = pAdapter->pVARXDesc;
	LPCPS_ETH_RX_STATUS pRxStatus = pAdapter->pVARXStatus;

	for (i = 0; i < pAdapter->dwRxStrides; i++)
	{
		pRxDescriptors[i].PacketAddress	= pAdapter->pPARXBuffer + (i*MAX_FRAME_SIZE);
		pRxDescriptors[i].Control		= ETH_RX_DESC_INTERRUPT |
										  (MAX_FRAME_SIZE-1);

		pRxStatus[i].StatusInfo			= 0;
		pRxStatus[i].StatusHashCRC		= 0;
	}

	//	Set EMAC Receive Descriptor Registers.	PHYSICAL VALUE ONLY
	pAdapter->pEMACRegs->rxdescriptor		= pAdapter->pPARXDesc;
	pAdapter->pEMACRegs->rxstatus			= pAdapter->pPARXStatus;
	pAdapter->pEMACRegs->rxdescriptornumber	= pAdapter->dwRxStrides - 1;

	//	Rx Descriptors Point to 0
	pAdapter->pEMACRegs->rxconsumeindex		= 0;
}


//	function initializes Tx Descriptors
static void tx_descr_init (PMINIPORT_ADAPTER pAdapter)
{
	DWORD					i;
	LPCPS_ETH_DESC			pTxDescriptors = pAdapter->pVATXDesc;
	LPCPS_ETH_TX_STATUS		pTxStatus = pAdapter->pVATXStatus;

	for (i = 0; i < pAdapter->dwTxStrides; i++)
	{
		pTxDescriptors[i].PacketAddress	= pAdapter->pPATXBuffer + (i*MAX_FRAME_SIZE);
		pTxDescriptors[i].Control		= 0;

		pTxStatus[i].StatusInfo			= 0;
	}

	//	Set EMAC Transmit Descriptor Registers. PHYSICAL VALUE ONLY
	pAdapter->pEMACRegs->txdescriptor		= pAdapter->pPATXDesc;
	pAdapter->pEMACRegs->txstatus			= pAdapter->pPATXStatus;
	pAdapter->pEMACRegs->txdescriptornumber	= pAdapter->dwTxStrides - 1;

	//	Tx Descriptors Point to 0
	pAdapter->pEMACRegs->txproduceindex		= 0;
}

/*	void EMAC_InitEngine()

	Brief:	This function initialize the DMA engine
*/
void EMAC_InitEngine(PMINIPORT_ADAPTER pAdapter)
{
	rx_descr_init(pAdapter);
	tx_descr_init(pAdapter);
}

/*
	Brief:	This function returns TRUE when a frame has been received
*/
BOOL EMAC_FrameRxd(PMINIPORT_ADAPTER pAdapter)
{
	return (pAdapter->pEMACRegs->rxproduceindex != pAdapter->pEMACRegs->rxconsumeindex);     // more packets received ?
}

LPCPS_ETH_RX_STATUS EMAC_RxStatusGetCurrent(PMINIPORT_ADAPTER pAdapter)
{
	return &pAdapter->pVARXStatus[pAdapter->pEMACRegs->rxconsumeindex];
}

LPCPS_ETH_DESC EMAC_RxDescGetCurrent(PMINIPORT_ADAPTER pAdapter)
{
	return &pAdapter->pVARXDesc[pAdapter->pEMACRegs->rxconsumeindex];
}

void EMAC_RxDescConsume(PMINIPORT_ADAPTER pAdapter)
{
	//	Clear the status of the Descriptor
	pAdapter->pVARXStatus[pAdapter->pEMACRegs->rxconsumeindex].StatusInfo		= 0;
	pAdapter->pVARXStatus[pAdapter->pEMACRegs->rxconsumeindex].StatusHashCRC	= 0;

	//	Increment Consume Index
	pAdapter->pEMACRegs->rxconsumeindex = (pAdapter->pEMACRegs->rxconsumeindex + 1) % (pAdapter->pEMACRegs->rxdescriptornumber + 1);
}

void EMAC_RxDiscardFrame(PMINIPORT_ADAPTER pAdapter)
{
	LPCS_ETH_RX_STATUS RXStatus = pAdapter->pVARXStatus[pAdapter->pEMACRegs->rxconsumeindex];

	//	We parse the complete frame
	while(1)
	{
		EMAC_RxDescConsume(pAdapter);	//	We discard the current packet

		if((RXStatus.StatusInfo & ETH_RX_STATUS_INFO_LASTFLAG) != 0)	//	Was is the last one of the frame?
			break;	//	If so, then we can exit

		while(!EMAC_FrameRxd(pAdapter));	//	Otherwise, make sure we have another packet

		RXStatus = pAdapter->pVARXStatus[pAdapter->pEMACRegs->rxconsumeindex];	//	And get the new status
	}
}

void EMAC_RxBuffCopyCurrent(PMINIPORT_ADAPTER pAdapter, BYTE *pbData, UINT16 wLength)
{
	memcpy(pbData,RX_BUF(pAdapter->pVARXBuffer, pAdapter->pEMACRegs->rxconsumeindex),wLength);
}

/*
	Brief:	This function returns if the DMA engine is ready to start transmitting
*/
BOOL EMAC_FrameTxRdy(PMINIPORT_ADAPTER pAdapter)
{
	UINT16	wNextPos = (UINT16)((pAdapter->pEMACRegs->txproduceindex + 1) % (pAdapter->pEMACRegs->txdescriptornumber + 1));

	if(wNextPos != pAdapter->pEMACRegs->txconsumeindex)
		return TRUE;
	else
		return FALSE;
}

LPCPS_ETH_TX_STATUS EMAC_TxStatusGetCurrent(PMINIPORT_ADAPTER pAdapter)
{
	return &pAdapter->pVATXStatus[pAdapter->pEMACRegs->txproduceindex];
}

LPCPS_ETH_DESC EMAC_TxDescGetCurrent(PMINIPORT_ADAPTER pAdapter)
{
	return &pAdapter->pVATXDesc[pAdapter->pEMACRegs->txproduceindex];
}

LPCPS_ETH_TX_STATUS EMAC_TxStatusGetIndex(PMINIPORT_ADAPTER pAdapter, DWORD dwIndex)
{
	return &pAdapter->pVATXStatus[dwIndex];
}

LPCPS_ETH_DESC EMAC_TxDescGetIndex(PMINIPORT_ADAPTER pAdapter, DWORD dwIndex)
{
	return &pAdapter->pVATXDesc[dwIndex];
}

void EMAC_TxDescProduce(PMINIPORT_ADAPTER pAdapter)
{
	//	Increment Production Index

	pAdapter->pEMACRegs->txproduceindex = ((pAdapter->pEMACRegs->txproduceindex + 1) % (pAdapter->pEMACRegs->txdescriptornumber + 1));
}

void EMAC_TxDescProduceAndWait(PMINIPORT_ADAPTER pAdapter)
{
	EMAC_TxDescProduce(pAdapter);

	//	Loop while we haven't finished to send it
	while((pAdapter->pEMACRegs->status & STATUS_TX) != 0)
		;
}

DWORD EMAC_TxSlotGetCurrentProducer(PMINIPORT_ADAPTER pAdapter)
{
	return pAdapter->pEMACRegs->txproduceindex;
}
DWORD EMAC_TxSlotGetLastConsumed(PMINIPORT_ADAPTER pAdapter)
{
	return pAdapter->pEMACRegs->txproduceindex;
}

void EMAC_TxBuffCopyCurrent(PMINIPORT_ADAPTER pAdapter, PNDIS_PACKET pNDISPacket)
{
	PNDIS_BUFFER	pCurrentBuffer;
	UINT			PacketLength, CurrentBufferLength;
	PUCHAR			pCurrentBufferAddr, ptemp;

	//Query for the first non-zero buffer
	NdisQueryPacket(pNDISPacket,NULL,NULL,&pCurrentBuffer,(PUINT) &PacketLength);

	NdisQueryBuffer(pCurrentBuffer, (PVOID*)&pCurrentBufferAddr, &CurrentBufferLength);
	while ((pCurrentBuffer) && (CurrentBufferLength == 0))
	{
		NdisGetNextBuffer(pCurrentBuffer, (PNDIS_BUFFER *)&pCurrentBuffer);
		NdisQueryBuffer(pCurrentBuffer, (PVOID*)&pCurrentBufferAddr, &CurrentBufferLength);
	}

	//Copy the data to the TxBuffer   
	ptemp = TX_BUF(pAdapter->pVATXBuffer, pAdapter->pEMACRegs->txproduceindex);
	do
	{
		NdisMoveMemory(ptemp, pCurrentBufferAddr, (ULONG)CurrentBufferLength);
		(ULONG)ptemp += CurrentBufferLength;
		NdisGetNextBuffer (pCurrentBuffer, (PNDIS_BUFFER *)&pCurrentBuffer);  //Get the next buffer
		NdisQueryBuffer (pCurrentBuffer, (PVOID*)&pCurrentBufferAddr, &CurrentBufferLength);        
	}
	while(pCurrentBuffer);
}
