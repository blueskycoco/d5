#ifndef __LPC3250_NDIS_ADAPTER__
#define __LPC3250_NDIS_ADAPTER__

#include "Lpc32xx_mac.h"

//#define NDIS50_MINIPORT 1 //jj
//NDIS Version Declaration
#ifdef NDIS50_MINIPORT
#define DRIVER_NDIS_MAJOR_VERSION	5	
#else
#define DRIVER_NDIS_MAJOR_VERSION	4
#endif
#define DRIVER_NDIS_MINOR_VERSION	0
#define DRIVER_NDIS_VERSION			((DRIVER_NDIS_MAJOR_VERSION << 8) + DRIVER_NDIS_MINOR_VERSION)

#define DRIVER_BUILD_NUM			0

#define DEBUGMASK(n)		(0x00000001 << n)

#define MASK_INIT			DEBUGMASK(0)
#define MASK_DEINIT			DEBUGMASK(1)
#define MASK_TX				DEBUGMASK(2)
#define MASK_RX				DEBUGMASK(3)
#define MASK_INTR			DEBUGMASK(4)
#define MASK_ZONE5			DEBUGMASK(5)
#define MASK_ZONE6			DEBUGMASK(6)
#define MASK_ZONE7			DEBUGMASK(7)
#define MASK_ZONE8			DEBUGMASK(8)
#define MASK_ZONE9			DEBUGMASK(9)
#define MASK_ZONE10			DEBUGMASK(10)
#define MASK_ZONE11			DEBUGMASK(11)
#define MASK_ZONE12			DEBUGMASK(12)
#define MASK_INFO			DEBUGMASK(13)
#define MASK_WARN			DEBUGMASK(14)
#define MASK_ERROR			DEBUGMASK(15)

#ifdef DEBUG
// These macros are used as the first arg to DEBUGMSG.
	#define ZONE_INIT		DEBUGZONE(0)
	#define ZONE_DEINIT		DEBUGZONE(1)
	#define ZONE_TX			DEBUGZONE(2)
	#define ZONE_RX			DEBUGZONE(3)
	#define ZONE_INTR		DEBUGZONE(4)
	#define ZONE_ZONE5		DEBUGZONE(5)
	#define ZONE_ZONE6		DEBUGZONE(6)
	#define ZONE_ZONE7		DEBUGZONE(7)
	#define ZONE_ZONE8		DEBUGZONE(8)
	#define ZONE_ZONE9		DEBUGZONE(9)
	#define ZONE_ZONE10		DEBUGZONE(10)
	#define ZONE_ZONE11		DEBUGZONE(11)
	#define ZONE_ZONE12		DEBUGZONE(12)
	#define ZONE_INFO		DEBUGZONE(13)
	#define ZONE_WARN		DEBUGZONE(14)
	#define ZONE_ERROR		DEBUGZONE(15)
#else
  // For release configurations, these conditionals are always 0.
	#define ZONE_INIT		0
	#define ZONE_DEINIT		0
	#define ZONE_TX			0
	#define ZONE_RX			0
	#define ZONE_INTR		0
	#define ZONE_ZONE5		0
	#define ZONE_ZONE6		0
	#define ZONE_ZONE7		0
	#define ZONE_ZONE8		0
	#define ZONE_ZONE9		0
	#define ZONE_ZONE10		0
	#define ZONE_ZONE11		0
	#define ZONE_ZONE12		0
	#define ZONE_INFO		0
	#define ZONE_WARN		0
	#define ZONE_ERROR		0
#endif // DEBUG

//----------------------------------------------------------------


typedef UCHAR	tMAC_Address[ETH_LENGTH_OF_ADDRESS];

//Multicast Table Entry Structure
#define MAX_MULTICAST_ADDRESS   128

typedef struct 
{
	UINT  MulticastTableEntryCount;
	UCHAR MulticastTableEntry[MAX_MULTICAST_ADDRESS * ETH_LENGTH_OF_ADDRESS];
} MULTICAST_TABLE;

typedef struct _TX_PACKET
{
	PNDIS_PACKET	pNDISPacket;	//	NDIS protocol Packet
	DWORD			dwPacketSize;	//	Size of the packet
} TX_PACKET,*PTX_PACKET;


#define MINIPORT_ADAPTER_SIZE		(sizeof(MINIPORT_ADAPTER))
#define ETHERNET_HEADER_SIZE		(14)
#define MAX_FRAME_SIZE				(1520)
#define MAX_FRAME_DATA_SIZE			(MAX_FRAME_SIZE - ETHERNET_HEADER_SIZE)
#define MIN_FRAME_SIZE				(ETHERNET_HEADER_SIZE)
#define MIN_LEGAL_FRAME_SIZE		(64)
#define MAC_ADDRESS_SIZE			(6)


//Defines possible states for MAC structures.
typedef enum _EMAC_STATE_TYPE
{
		VOID_STATE,                        // Illegal value.
		INITIALIZING_STATE,                // Structure is being built.
		NORMAL_STATE,                      // Operational state.		
		RESET_STATE,
		LINKDOWN_STATE
} EMAC_STATE_TYPE;

//--------------------- DRIVER STRUCTURE --------------------------------
#pragma pack(4)
typedef  struct _MINIPORT_ADAPTER
{
	NDIS_HANDLE				hAdapter;

	DWORD					dwBufferPhyAddr;
	DWORD					dwTxStrides;
	DWORD					dwRxStrides;
	DWORD					dwIRQNumber;
	DWORD					dwControllerAddress;
	BOOL					bRMII;

	EMAC_STATE_TYPE			eState;					//	Adapter Status
	BOOL					bMediaConnected;		//  Link Status
	BOOL					bAutoNeg;				//	Auto Negotation Enabled ?
	BOOL					b100Mbps;				//	100 Mbps?
	BOOL					bFullDuplex;			//  FullDuplex/nHalfDuplex ?

	MULTICAST_TABLE			tMulticastTable;		//	Multicast Table
	BOOL					bPerfectMatch;			//	Do we forward the frame for this station?
	BOOL					bPromiscuous;			//	Do we forward all the frames up with no filtering?
	BOOL					bRxBroadcast;			//	Do we forward the Broadcasted frames up?
	BOOL					bRxMulticastHash;		//	Do we forward matched multicast frames?
	BOOL					bRxMulticastAll;		//	Do we forward all multicast frames?
	BOOL					bRxUnicastHash;			//	Do we forward matched unicast frames?
	BOOL					bRxUnicastAll;			//	Do we forward all unicast frames?

	P_ETHERNET_REGS_T			pEMACRegs;				//	EMAC Mapped Registers
	DWORD					dwEMACBuffersSize;		//	Length to allocate
	DWORD					pPATXDesc, pPARXDesc;	//	Physical Addresses of the TX and RX descriptors
	DWORD					pPATXStatus, pPARXStatus;//	Physical Addresses of the TX and RX status
	DWORD					pPATXBuffer, pPARXBuffer;//	Physical Addresses pf the TX and RX DMA buffers
	PVOID					pVAEMACBuffers;			//	Pointers to the EMAC DMA buffers.
	LPCPS_ETH_DESC			pVATXDesc, pVARXDesc;	//	Pointers to TX and RX descriptors
	LPCPS_ETH_TX_STATUS		pVATXStatus;			//	Pointers to TX and RX status
	LPCPS_ETH_RX_STATUS		pVARXStatus;
	PBYTE					pVARXBuffer;
	PBYTE					pVATXBuffer;

	PBYTE					pVALookAheadBuffer;		//	Pointer to lookahead buffer (communication with NDIS layer for received packet);
	DWORD					dwLookAheadBufferSize;	//	Size of the Lookahead buffer
	PTX_PACKET				pTXPackets;				//	Transmission Packets
	DWORD					dwLastConsumed;			//	Last packet # treated in the IT

	BOOLEAN					IsInterruptSet;			//	Attached to interrupt using NdisMRegisterInterrupt(...)
	BOOLEAN					IsPortRegistered;		//	I/O Port registered with NdisMRegisterIORage (...)
	NDIS_MINIPORT_INTERRUPT InterruptInfo;			//	From NdisMRegisterInterrupt(..)

	tMAC_Address			MACAddress;			//	Current Station address

//    MINIPORT_PACKET_QUE     AckPending;         //  Waiting completion intr
//	PUCHAR                  TxBuffer;           //  Pointer to the TX Copy buffer

//	MINIPORT_PACKET_QUE     AllocPending;
//	BOOLEAN					AllocIntPending;

//	//Required Statistics.
//	UINT					Stat_TxOK;
//	UINT					Stat_RxOK;
//	UINT					Stat_TxError;
//	UINT					Stat_RxError;
//	UINT					Stat_RxOvrn;
//	UINT					Stat_AlignError;
//	UINT					Stat_SingleColl;
//	UINT					Stat_MultiColl;
//#ifdef ADENEO_
//	USHORT                  TXCounter;          // Counter for packets currently being processed
//#endif
}MINIPORT_ADAPTER, *PMINIPORT_ADAPTER;
#pragma pack()

//------------------------ GLOBAL OID ------------------------------
//Supported OIDs
static  const NDIS_OID GlobalObjects[] = 
{
    OID_GEN_SUPPORTED_LIST,
    OID_GEN_MEDIA_SUPPORTED,
    OID_GEN_MEDIA_IN_USE,
    OID_GEN_MAXIMUM_LOOKAHEAD,
    OID_GEN_MAXIMUM_FRAME_SIZE,
    OID_GEN_LINK_SPEED,
    OID_GEN_TRANSMIT_BUFFER_SPACE,
    OID_GEN_RECEIVE_BUFFER_SPACE,
    OID_GEN_TRANSMIT_BLOCK_SIZE,
    OID_GEN_RECEIVE_BLOCK_SIZE,
    OID_GEN_VENDOR_ID,
    OID_GEN_VENDOR_DESCRIPTION,
    OID_GEN_CURRENT_LOOKAHEAD,
    OID_GEN_DRIVER_VERSION,
    OID_GEN_MAXIMUM_TOTAL_SIZE,
    OID_GEN_MAC_OPTIONS,
	OID_GEN_HARDWARE_STATUS,
	OID_GEN_CURRENT_PACKET_FILTER,
    OID_802_3_PERMANENT_ADDRESS,
    OID_802_3_CURRENT_ADDRESS,
    OID_802_3_MULTICAST_LIST,
    OID_802_3_MAXIMUM_LIST_SIZE,
//	OID_802_3_RCV_ERROR_ALIGNMENT,
//	OID_802_3_XMIT_ONE_COLLISION,
//	OID_802_3_XMIT_MORE_COLLISIONS,
    OID_GEN_MAXIMUM_SEND_PACKETS,
	OID_GEN_VENDOR_DRIVER_VERSION,
	OID_GEN_MEDIA_CONNECT_STATUS,
//	OID_GEN_XMIT_OK,
//	OID_GEN_RCV_OK,
//	OID_GEN_XMIT_ERROR,
//	OID_GEN_RCV_ERROR,
//	OID_GEN_RCV_NO_BUFFER
};
extern ETHERNET_REGS_T *p_ETH_REGS;

//OID Return Strings
//OID Query return strings.
#define DRV_VENDOR_NAME			"NXP LPC3250 NDIS Ethernet Adapter"
#define SIZE_DRV_VENDOR_NAME    sizeof(DRV_VENDOR_NAME)
	
#endif __LPC3250_NDIS_ADAPTER__