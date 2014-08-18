
#ifndef __LPC3250_NDIS_PROTO__
#define __LPC3250_NDIS_PROTO__

#include <NDIS.H>

VOID		LPC3xxx_NDIS_MiniportHalt(NDIS_HANDLE MiniportAdapterContext);

VOID		LPC3xxx_NDIS_MiniPortHandleInterrupt(IN NDIS_HANDLE  AdapterContext);


NDIS_STATUS	LPC3xxx_MiniportInitialize(	PNDIS_STATUS	OpenErrorStatus,
										PUINT			SelectedMediumIndex,
										PNDIS_MEDIUM	MediumArray,
										UINT			MediumArraySize,
										NDIS_HANDLE		MiniportAdapterHandle,
										NDIS_HANDLE		WrapperConfigurationContext);

VOID		LPC3xxx_NDIS_MiniportISR(	PBOOLEAN    InterruptRecognized,
										PBOOLEAN    QueueMiniportHandleInterrupt,
										NDIS_HANDLE MiniportAdapterContext);


NDIS_STATUS	LPC3xxx_NDIS_MiniportQueryInformation(	NDIS_HANDLE AdapterContext,
														NDIS_OID    Oid,
														PVOID       InformationBuffer,
														ULONG       InformationBufferLength,
														PULONG      BytesWritten,
														PULONG      BytesNeeded);

NDIS_STATUS	LPC3xxx_NDIS_MiniportReset		(PBOOLEAN    AddressingReset,
											NDIS_HANDLE AdapterContext);

NDIS_STATUS	LPC3xxx_NDIS_MiniportSend		(NDIS_HANDLE	AdapterContext,
											PNDIS_PACKET	Packet,
											UINT			Flags);

NDIS_STATUS LPC3xxx_NDIS_MiniportSetInformation(NDIS_HANDLE MiniportAdapterContext,
												NDIS_OID Oid,
												PVOID InformationBuffer,
												ULONG InformationBufferLength,
												PULONG BytesRead,
												PULONG BytesNeeded);

//NDIS_STATUS LPC3xxx_NDIS_MiniportTransferData(
//											PNDIS_PACKET Packet,
//											PUINT BytesTransferred,
//											NDIS_HANDLE MiniportAdapterContext,
//											NDIS_HANDLE MiniportReceiveContext,
//											UINT ByteOffset,
//											UINT BytesToTransfer
//											);

VOID LPC3xxx_NDIS_MiniportEnableInterrupt(NDIS_HANDLE MiniportAdapterContext);

VOID LPC3xxx_NDIS_MiniportDisableInterrupt(NDIS_HANDLE MiniportAdapterContext);


//BOOLEAN		EstablishLink					(MINIPORT_ADAPTER *Adapter);
//
//VOID		DumpRegisters					(MINIPORT_ADAPTER *Adapter);

NDIS_STATUS MiniPortChangeFilter			(PMINIPORT_ADAPTER	pAdapter,
											 NDIS_OID			NewFilter);

NDIS_STATUS MiniPortChangeAddresses			(PMINIPORT_ADAPTER	pAdapter,
											 UCHAR            *AddressList,
											 UINT              AddressCount);

//BOOLEAN		CheckMultiCastAddress			(
//											 USHORT *ReadBuffer, 
//											 MINIPORT_ADAPTER *Adapter);

BOOLEAN		LPC3xxx_NDIS_MiniportCheckforHang(NDIS_HANDLE AdapterContext);

VOID		LPC3xxx_NDIS_MiniportUpdateLink(PMINIPORT_ADAPTER	pAdapter);





NDIS_STATUS	CopyInfo(UCHAR *, UCHAR *, UINT, UINT, UINT  *, UINT  *);
void		BackOut(PMINIPORT_ADAPTER pAdapter);
BOOL		HardReset(PMINIPORT_ADAPTER pAdapter);
BOOL		AdapterReset(PMINIPORT_ADAPTER pAdapter);


//------------------------------------------------------------------------------
//	PHY functions (BSP Specific)
//------------------------------------------------------------------------------
BOOL	PHY_CheckLinkOK(P_ETHERNET_REGS_T pEthernet);
BOOL	PHY_FullDuplex(P_ETHERNET_REGS_T pEthernet);
BOOL	PHY_Speed(P_ETHERNET_REGS_T pEthernet);
void	PHY_HWReset(P_ETHERNET_REGS_T pEthernet);
BOOL	PHY_SWReset(P_ETHERNET_REGS_T pEthernet);
BOOL	PHY_CheckCompatibility(P_ETHERNET_REGS_T pEthernet);
BOOL	PHY_InitLink(P_ETHERNET_REGS_T pEthernet);
void	PHY_SetInterface(P_ETHERNET_REGS_T pEthernet, BOOL bRMII);
void	PHY_SetSpeed(P_ETHERNET_REGS_T pEthernet,BOOL b100Mbps);
void	PHY_SetDuplex(P_ETHERNET_REGS_T pEthernet,BOOL bFullDuplex);

//	EMAC DMA functions
//		init
void				EMAC_InitEngine(PMINIPORT_ADAPTER pAdapter);
//		copy buffer from/to DMA engine
void				EMAC_TxBuffCopyCurrent(PMINIPORT_ADAPTER pAdapter, PNDIS_PACKET pNDISPacket);
void				EMAC_RxBuffCopyCurrent(PMINIPORT_ADAPTER pAdapter, BYTE *pbData, UINT16 wLength);
//		DMA engine ready functions
BOOL				EMAC_FrameTxRdy(PMINIPORT_ADAPTER pAdapter);
BOOL				EMAC_FrameRxd(PMINIPORT_ADAPTER pAdapter);

LPCPS_ETH_DESC		EMAC_TxDescGetCurrent(PMINIPORT_ADAPTER pAdapter);
LPCPS_ETH_TX_STATUS	EMAC_TxStatusGetCurrent(PMINIPORT_ADAPTER pAdapter);
LPCPS_ETH_RX_STATUS EMAC_RxStatusGetCurrent(PMINIPORT_ADAPTER pAdapter);

void				EMAC_TxDescProduce(PMINIPORT_ADAPTER pAdapter);
void				EMAC_TxDescProduceAndWait(PMINIPORT_ADAPTER pAdapter);
void				EMAC_RxDiscardFrame(PMINIPORT_ADAPTER pAdapter);
void				EMAC_RxDescConsume(PMINIPORT_ADAPTER pAdapter);

DWORD				EMAC_TxSlotGetCurrentProducer(PMINIPORT_ADAPTER pAdapter);
DWORD				EMAC_TxSlotGetLastConsumed(PMINIPORT_ADAPTER pAdapter);

LPCPS_ETH_TX_STATUS	EMAC_TxStatusGetIndex(PMINIPORT_ADAPTER pAdapter, DWORD dwIndex);
LPCPS_ETH_DESC		EMAC_TxDescGetIndex(PMINIPORT_ADAPTER pAdapter, DWORD dwIndex);


void LPC_PrintFrame(BYTE *pbData, DWORD dwLength);
void LPC_PrintTSV(P_ETHERNET_REGS_T pEMAC);
void LPC_PrintRSV(P_ETHERNET_REGS_T pEMAC);

#endif