#include <Ndis.h>
#include <PKFuncs.h>
#include <nkintr.h>

#include "LPC3250_NDIS_Adapter.h"
#include "LPC3250_NDIS_Proto.h"
//#include "interrupt.h"
#include "lpc32xx_intc.h"
#include "lpc32xx_chip.h"

#include "clkpwr_support.h"
#include "MapRegisters.h"
//#include "lpcddk.h"

extern NDIS_PHYSICAL_ADDRESS HighestAcceptedMax;

CLKPWR_REGS_T *n_pCLKPWRRegs;

void        BackOut(MINIPORT_ADAPTER *Adapter);
NDIS_STATUS GetRegistrySettings(MINIPORT_ADAPTER *, NDIS_HANDLE);
/*
 Function Name : 	LPC3xxx_MiniportInitialize
 Description   :	
					Called by the NDIS Wrapper to initialize the adapter. 
						0. Verify the Adapter v/s the driver
						1. Create and initilize the adapter structure
						2. Read and load the registry settings
						3. Initialize the chip
						4. Establish the link
 Parameters    :
					PNDIS_STATUS OpenErrorStatus - Additional error status, if return value is error
					       PUINT MediumIndex	 - specifies the medium type the driver or its network adapter uses
					PNDIS_MEDIUM MediumArray	 - Specifies an array of NdisMediumXXX values from which 
													MiniportInitialize selects one that its network adapter supports 
													or that the driver supports as an interface to higher-level drivers. 

						    UINT MediumArraySize - Specifies the number of elements at MediumArray
					 NDIS_HANDLE AdapterHandle   - Specifies a handle identifying the miniport’s network adapter, 
													which is assigned by the NDIS library
					 NDIS_HANDLE ConfigurationContext - Specifies a handle used only during initialization for 
														 calls to NdisXXX configuration and initialization functions

 Return Value  :
					NDIS_STATUS		Status
			
*/
NDIS_STATUS LPC3xxx_MiniportInitialize(	PNDIS_STATUS	pOpenErrorStatus,
										PUINT			pMediumIndex,
										PNDIS_MEDIUM	pMediumArray,
										UINT			nMediumArraySize,
										NDIS_HANDLE		hAdapterHandle,
										NDIS_HANDLE		hConfigurationContext)
{
	NDIS_STATUS         Status = NDIS_STATUS_SUCCESS;
	UINT                nArrayIndex;
	PMINIPORT_ADAPTER   pAdapter;
	PHYSICAL_ADDRESS pa;

	DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS ==> MiniportInitialize\r\n")));

	//Check for the supported 802.3 media
	for(nArrayIndex = 0; nArrayIndex < nMediumArraySize; nArrayIndex++)
	{
		if(NdisMedium802_3 == pMediumArray[nArrayIndex])
			break;
	}

    if(nArrayIndex == nMediumArraySize)
    {
		DEBUGMSG(ZONE_INIT | ZONE_ERROR, (TEXT("LPC3xxx NDIS :  ERROR - No Supported Media types \r\n")));
        DEBUGMSG(ZONE_INIT | ZONE_ERROR, (TEXT("LPC3xxx NDIS <== MiniportInitialize  \r\n")));
        return(NDIS_STATUS_UNSUPPORTED_MEDIA);
    }

    *pMediumIndex = nArrayIndex;

	//Allocate memory for the adapter structure
	Status = NdisAllocateMemory((PVOID *) &pAdapter, MINIPORT_ADAPTER_SIZE, 0, HighestAcceptedMax);

    if(Status != NDIS_STATUS_SUCCESS)
    {
        DEBUGMSG(ZONE_INIT | ZONE_ERROR, (TEXT("LPC3xxx NDIS:   ERROR - No Memory for Adapter Structure!\r\n")));
		DEBUGMSG(ZONE_INIT | ZONE_ERROR, (TEXT("LPC3xxx NDIS <== MiniPort Initialize\r\n")));
        return(NDIS_STATUS_RESOURCES);
    }
    NdisZeroMemory(pAdapter, MINIPORT_ADAPTER_SIZE); //Clean it up

//Set up the default values
	pAdapter->hAdapter			= hAdapterHandle;
	pAdapter->eState			= INITIALIZING_STATE;
	pAdapter->IsInterruptSet	= FALSE;
	pAdapter->IsPortRegistered	= FALSE;

	pAdapter->dwIRQNumber		= -1;
	pAdapter->bMediaConnected	= FALSE;
	pAdapter->bAutoNeg			= TRUE;		//	Autoneg per default (overriden by registry settings)
	pAdapter->bFullDuplex		= TRUE;		//	Full duplex per default
	pAdapter->b100Mbps			= TRUE;		//	100 Mbps per default
	pAdapter->bRMII				= TRUE;		//	RMII interface per default

	//Setup Receive control Register
	pAdapter->bPerfectMatch		= TRUE;		//	We want to receive the frames for this station
	pAdapter->bPromiscuous		= FALSE;	//	Not in Promescuous mode per default
	pAdapter->bRxBroadcast		= TRUE;		//	We do forward the broadcasted frames (DHCP required at least)
	pAdapter->bRxMulticastHash	= TRUE;		//	We forward matched multicast frames
	pAdapter->bRxMulticastAll	= FALSE;	//	We do not forward all multicast frames
	pAdapter->bRxUnicastHash	= TRUE;	//	We do not forward matched unicast frames
	pAdapter->bRxUnicastAll		= TRUE;	//	We do not forward all unicast frames

	pAdapter->dwBufferPhyAddr	= 0;
	pAdapter->dwTxStrides		= 0;
	pAdapter->dwRxStrides		= 0;
	pAdapter->dwControllerAddress	= 0;

	pAdapter->pEMACRegs		= NULL;
	pAdapter->dwEMACBuffersSize = 0;
	pAdapter->pPATXDesc		= 0;
	pAdapter->pPARXDesc		= 0;
	pAdapter->pPATXBuffer	= 0;
	pAdapter->pPARXBuffer	= 0;
	pAdapter->pPATXStatus	= 0;
	pAdapter->pPARXStatus	= 0;
	pAdapter->pVAEMACBuffers= NULL;
	pAdapter->pVATXDesc		= NULL;
	pAdapter->pVARXDesc		= NULL;
	pAdapter->pVATXStatus	= NULL;
	pAdapter->pVARXStatus	= NULL;

	//Allocate memory for the LookAhead buffer
	pAdapter->dwLookAheadBufferSize = MAX_FRAME_SIZE;
	Status = NdisAllocateMemory((PVOID *) &pAdapter->pVALookAheadBuffer, pAdapter->dwLookAheadBufferSize, 0, HighestAcceptedMax);

    if(Status != NDIS_STATUS_SUCCESS)
    {
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS: ERROR - No Memory for LookAhead buffer!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== MiniPort Initialize\r\n")));
        BackOut(pAdapter);
        return(NDIS_STATUS_RESOURCES);
    }
    NdisZeroMemory(pAdapter->pVALookAheadBuffer, pAdapter->dwLookAheadBufferSize); //Clean it up

	pAdapter->MACAddress[0]	= 0x02;
	pAdapter->MACAddress[1]	= 0x03;
	pAdapter->MACAddress[2]	= 0x04;
	pAdapter->MACAddress[3]	= 0x06;
	pAdapter->MACAddress[4]	= 0x06;
	pAdapter->MACAddress[5]	= 0x08;

	//Get the adapter information from the registry
	Status = GetRegistrySettings(pAdapter, hConfigurationContext);
	if(Status != NDIS_STATUS_SUCCESS)
    {
        DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS: ERROR - Configure Adapter failed!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== MiniPort Initialize\r\n")));
        BackOut(pAdapter);
        return(NDIS_STATUS_FAILURE);
    }

	//	Allocate the memory for Buffers

	//		Computing required space
	pAdapter->dwEMACBuffersSize = pAdapter->dwRxStrides * (	  sizeof(LPCS_ETH_DESC)
															+ sizeof(LPCS_ETH_RX_STATUS)
															+ MAX_FRAME_SIZE)
								+ pAdapter->dwTxStrides * (	  sizeof(LPCS_ETH_DESC) 
															+ sizeof(LPCS_ETH_TX_STATUS)
															+ MAX_FRAME_SIZE);

	//		Allocating space
	pAdapter->pVAEMACBuffers = MapRegisters (pAdapter->dwBufferPhyAddr,
											pAdapter->dwEMACBuffersSize);

	if(pAdapter->pVAEMACBuffers == NULL)
    {
        RETAILMSG(1, (TEXT("LPC3xxx NDIS:ERROR : Can't Allocate Buffer!\r\n")));
		BackOut(pAdapter);
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:<== MiniPort Initialize FAILED !!\r\n")));
        return(NDIS_STATUS_RESOURCES);
    }

	DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:Allocated DMA Buffers Successfully!\r\n")));

	//		Splitting allocated buffers into smaller areas
	//			Physical Addresses
	pAdapter->pPARXDesc		= pAdapter->dwBufferPhyAddr;
	pAdapter->pPARXStatus	= pAdapter->pPARXDesc	+ pAdapter->dwRxStrides * sizeof(LPCS_ETH_DESC);
	pAdapter->pPATXDesc		= pAdapter->pPARXStatus + pAdapter->dwRxStrides * sizeof(LPCS_ETH_RX_STATUS);
	pAdapter->pPATXStatus	= pAdapter->pPATXDesc	+ pAdapter->dwTxStrides * sizeof(LPCS_ETH_DESC);
	pAdapter->pPARXBuffer	= pAdapter->pPATXStatus	+ pAdapter->dwTxStrides * sizeof(LPCS_ETH_TX_STATUS);
	pAdapter->pPATXBuffer	= pAdapter->pPARXBuffer	+ pAdapter->dwRxStrides * MAX_FRAME_SIZE;
	//			Virtual Addresses
	pAdapter->pVARXDesc		= pAdapter->pVAEMACBuffers;
	pAdapter->pVARXStatus	= (PVOID)((DWORD)pAdapter->pVARXDesc	+ pAdapter->dwRxStrides * sizeof(LPCS_ETH_DESC));
	pAdapter->pVATXDesc		= (PVOID)((DWORD)pAdapter->pVARXStatus	+ pAdapter->dwRxStrides * sizeof(LPCS_ETH_RX_STATUS));
	pAdapter->pVATXStatus	= (PVOID)((DWORD)pAdapter->pVATXDesc	+ pAdapter->dwTxStrides * sizeof(LPCS_ETH_DESC));
	pAdapter->pVARXBuffer	= (PVOID)((DWORD)pAdapter->pVATXStatus	+ pAdapter->dwTxStrides * sizeof(LPCS_ETH_TX_STATUS));
	pAdapter->pVATXBuffer	= (PVOID)((DWORD)pAdapter->pVARXBuffer	+ pAdapter->dwRxStrides * MAX_FRAME_SIZE);

	//	Allocating the TX Packet buffer
	Status = NdisAllocateMemory((PVOID *) &pAdapter->pTXPackets, sizeof(TX_PACKET) * pAdapter->dwTxStrides, 0, HighestAcceptedMax);
    if(Status != NDIS_STATUS_SUCCESS)
    {
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS: ERROR - No Memory for TXBuffers buffer!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== MiniPort Initialize\r\n")));
        BackOut(pAdapter);
        return(NDIS_STATUS_RESOURCES);
    }
	NdisZeroMemory(pAdapter->pTXPackets, sizeof(TX_PACKET) * pAdapter->dwTxStrides); //Clean it up

	//Register the interrupt
	NdisMSetAttributes(pAdapter->hAdapter, (NDIS_HANDLE) pAdapter, TRUE, NdisInterfaceInternal);
	Status = NdisMRegisterInterrupt(&pAdapter->InterruptInfo,
									pAdapter->hAdapter,
									pAdapter->dwIRQNumber,
									0,
									TRUE,
									FALSE,
									NdisInterruptLatched);

	if(Status != NDIS_STATUS_SUCCESS)
    {
        DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS : ERROR - Can't Attach to Interrupt!\r\n")));
		BackOut(pAdapter);
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:<== MiniPort Initialize\r\n")));
        return(Status);
    }
	else
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS: Interrupt Registered !!! \r\n")));

	pAdapter->IsInterruptSet = TRUE;

	//Register the IOBase address. Modify this code according to the platform
	Status = NdisMRegisterIoPortRange((PVOID *) &(pAdapter->pEMACRegs),
			pAdapter->hAdapter,
			pAdapter->dwControllerAddress,
			sizeof(ETHERNET_REGS_T));
	if(Status != NDIS_STATUS_SUCCESS)
	{
		RETAILMSG(1, (TEXT("LAN91C9111 : ERROR - Can't Register I/O Port Range!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LAN91C9111 <== MiniPort Initialize\r\n")));
        BackOut(pAdapter);
		return(NDIS_STATUS_RESOURCES);
	}

	pAdapter->IsPortRegistered = TRUE;

	//EnablePeriphClock(ETHERNET);
	pa.QuadPart = CLK_PM_BASE;
	n_pCLKPWRRegs = (CLKPWR_REGS_T *)
		MmMapIoSpace(pa, sizeof (CLKPWR_REGS_T), FALSE);
	
	if (n_pCLKPWRRegs == NULL)
	{
		RETAILMSG(1, (_T("LCD: lpc32xx_hw_init: Critcal error: cannot map registers!\r\n")));
		MmUnmapIoSpace(n_pCLKPWRRegs, sizeof(CLKPWR_REGS_T));
		return;
	}

	n_pCLKPWRRegs->clkpwr_macclk_ctrl = CLKPWR_MACCTRL_HRCCLK_EN|
										CLKPWR_MACCTRL_MMIOCLK_EN|
										CLKPWR_MACCTRL_DMACLK_EN|
										CLKPWR_MACCTRL_USE_RMII_PINS;
	//	//	Initialize Tx and Rx DMA Descriptors
	//DEBUGMSG(ZONE_INIT, (L"LPC_EthInit: Initializing Descriptor engine\r\n"));
	//EMAC_InitEngine(pAdapter);

	if(!HardReset(pAdapter))
	{
		return NDIS_STATUS_FAILURE;
	}

	DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== MiniportInitialize  \r\n")));

	//Ready

	return NDIS_STATUS_SUCCESS;
}

/*
 Function Name : 	GetRegistrySettings
 Description   :	
					Reads the registry values, and loads them into the adapter structure
 Parameters    :
					MINIPORT_ADAPTER *Adapter	- Pointer to the adapter structure
					NDIS_HANDLE       ConfigurationContext - Context handler, from the NDIS wrapper
			
 Return Value  :
					NDIS_STATUS		Status
*/
NDIS_STATUS GetRegistrySettings(MINIPORT_ADAPTER*	pAdapter,
								NDIS_HANDLE			hConfigurationContext)
{
	NDIS_STATUS						Status;
	NDIS_HANDLE						hConfiguration;
	PNDIS_CONFIGURATION_PARAMETER	pConfigurationParameter;
	BOOL							bSpeedDef = FALSE, bDuplexDef = FALSE;

	UCHAR*							pNewNetworkAddress = NULL;
	UINT							nNewNetworkAddressLength;

	//	EMAC specific settings
	NDIS_STRING						szBufferAddr	= NDIS_STRING_CONST("BufferAddr");
	NDIS_STRING						szTxStride		= NDIS_STRING_CONST("TxStrides");
	NDIS_STRING						szRxStride		= NDIS_STRING_CONST("RxStrides");
	NDIS_STRING						szITNum			= NDIS_STRING_CONST("IRQNumber");
	NDIS_STRING						szEMACAddr		= NDIS_STRING_CONST("IoBaseAddress");
	NDIS_STRING						szRMII			= NDIS_STRING_CONST("RMII");

	//	Eth Link settings
	NDIS_STRING						AutoNegString	= NDIS_STRING_CONST("AUTO-NEGOTIATION");
	NDIS_STRING						DuplexString	= NDIS_STRING_CONST("DUPLEX");
	NDIS_STRING						FullDupString	= NDIS_STRING_CONST("FULL");
	NDIS_STRING						HalfDupString	= NDIS_STRING_CONST("HALF");
	NDIS_STRING						SpeedString		= NDIS_STRING_CONST("SPEED");
	NDIS_STRING						Speed100String	= NDIS_STRING_CONST("100");
	NDIS_STRING						Speed10String	= NDIS_STRING_CONST("10");
	
	DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS ==> GetRegistrySettings\r\n")));

	//Open the Registry tree for this adapter.
    NdisOpenConfiguration(&Status, &hConfiguration, hConfigurationContext);
	if(Status != NDIS_STATUS_SUCCESS)
    {
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS : ERROR - No information for adapter!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== Configure Adapter\r\n")));
        return(NDIS_STATUS_FAILURE);
    }

	//Get configured Buffer address value from registry,
	NdisReadConfiguration(	&Status,   
							&pConfigurationParameter,
							hConfiguration,
							&szBufferAddr,
							NdisParameterInteger); 
	if(Status == NDIS_STATUS_SUCCESS)
	{
		pAdapter->dwBufferPhyAddr = (DWORD) pConfigurationParameter->ParameterData.IntegerData;
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:dwBufferPhyAddr    = 0x%.8x\r\n"), pAdapter->dwBufferPhyAddr));
	}
	else
	{
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS : ERROR - BufferAddr not in Registry!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== Configure Adapter\r\n")));
		NdisCloseConfiguration(hConfiguration);
		return(NDIS_STATUS_FAILURE);
	}

	//Get configured Number of TX Strides from registry,
	NdisReadConfiguration(	&Status,   
							&pConfigurationParameter,
							hConfiguration,
							&szTxStride,
							NdisParameterInteger); 
	if(Status == NDIS_STATUS_SUCCESS)
	{
		pAdapter->dwTxStrides = (DWORD) pConfigurationParameter->ParameterData.IntegerData;
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:dwTxStrides    = 0x%x\r\n"), pAdapter->dwTxStrides));
	}
	else
	{
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS : ERROR - TXStrides not in Registry!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== Configure Adapter\r\n")));
		NdisCloseConfiguration(hConfiguration);
		return(NDIS_STATUS_FAILURE);
	}

	//Get configured Number of RX Strides from registry,
	NdisReadConfiguration(	&Status,   
							&pConfigurationParameter,
							hConfiguration,
							&szRxStride,
							NdisParameterInteger); 
	if(Status == NDIS_STATUS_SUCCESS)
	{
		pAdapter->dwRxStrides = (DWORD) pConfigurationParameter->ParameterData.IntegerData;
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:dwRxStrides    = 0x%x\r\n"), pAdapter->dwRxStrides));
	}
	else
	{
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS : ERROR - RXStrides not in Registry!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== Configure Adapter\r\n")));
		NdisCloseConfiguration(hConfiguration);
		return(NDIS_STATUS_FAILURE);
	}

	//Get configured IRQ Number from registry,
	NdisReadConfiguration(	&Status,   
							&pConfigurationParameter,
							hConfiguration,
							&szITNum,
							NdisParameterInteger); 
	if(Status == NDIS_STATUS_SUCCESS)
	{
		pAdapter->dwIRQNumber = (DWORD) pConfigurationParameter->ParameterData.IntegerData;
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:dwIRQNumber    = 0x%x\r\n"), pAdapter->dwIRQNumber));
	}
	else
	{
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS : ERROR - IRQNumber not in Registry!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== Configure Adapter\r\n")));
		NdisCloseConfiguration(hConfiguration);
		return(NDIS_STATUS_FAILURE);
	}

	//Get configured EMAC base address from registry,
	NdisReadConfiguration(	&Status,   
							&pConfigurationParameter,
							hConfiguration,
							&szEMACAddr,
							NdisParameterInteger); 
	
	if(Status == NDIS_STATUS_SUCCESS)
	{
		pAdapter->dwControllerAddress = (DWORD) pConfigurationParameter->ParameterData.IntegerData;
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:dwControllerAddress    = 0x%.8x\r\n"), pAdapter->dwControllerAddress));
		RETAILMSG(1, (TEXT("LPC3xxx NDIS:dwControllerAddress    = 0x%.8x\r\n"), pAdapter->dwControllerAddress));
	}
	else
	{
		RETAILMSG(1, (TEXT("LPC3xxx NDIS : ERROR - IoBaseAddress not in Registry!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS : ERROR - IoBaseAddress not in Registry!\r\n")));
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== Configure Adapter\r\n")));
		NdisCloseConfiguration(hConfiguration);
		return(NDIS_STATUS_FAILURE);
	}

	
	//Get PHY interface type (MII/RMII) from registry,
	NdisReadConfiguration(	&Status,   
							&pConfigurationParameter,
							hConfiguration,
							&szRMII,
							NdisParameterInteger); 
	if(Status == NDIS_STATUS_SUCCESS)
	{
		pAdapter->bRMII = ((USHORT) pConfigurationParameter->ParameterData.IntegerData == 0) ? FALSE : TRUE;
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:bRMII    = %s\r\n"), pAdapter->bRMII ? L"TRUE" : L"FALSE"));
	}
	else
	{
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS : RMII not in Registry!\r\n")));
		pAdapter->bRMII = TRUE;
	}

	NdisReadConfiguration(	&Status,
							&pConfigurationParameter,
							hConfiguration,
							&DuplexString,
							NdisParameterString);
	
	if(Status == NDIS_STATUS_SUCCESS)
	{
		bDuplexDef = TRUE;

		if( NdisEqualString( (PNDIS_STRING) &pConfigurationParameter->ParameterData.StringData,&FullDupString, TRUE ) ) 
		{
			DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:Config is Full Duplex\r\n")));
			pAdapter->bFullDuplex = TRUE;
		}
		else if( NdisEqualString( (PNDIS_STRING) &pConfigurationParameter->ParameterData.StringData, &HalfDupString, TRUE ) ) 
		{
			DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:Config is Half Duplex\r\n")));
			pAdapter->bFullDuplex = FALSE;
		}				
	}

	NdisReadConfiguration(	&Status, 
							&pConfigurationParameter, 
							hConfiguration,
							&SpeedString,
							NdisParameterString);
	if(Status == NDIS_STATUS_SUCCESS) 
	{
		bSpeedDef = TRUE;
		if( NdisEqualString( (PNDIS_STRING) &pConfigurationParameter->ParameterData.StringData,(PNDIS_STRING) &Speed100String,TRUE ) ) 
		{
			DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:Config is 100 Mbps\r\n")));
			pAdapter->b100Mbps = TRUE;
		}
		else if( NdisEqualString( (PNDIS_STRING) &pConfigurationParameter->ParameterData.StringData,(PNDIS_STRING) &Speed10String,TRUE ) )
		{
			DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:Config is 10 Mbps\r\n")));
			pAdapter->b100Mbps = FALSE;
		}
	}

	if(bSpeedDef && bDuplexDef)
	{
		pAdapter->bAutoNeg = FALSE;
	}

	NdisReadConfiguration(	&Status,
							&pConfigurationParameter,
							hConfiguration,
							&AutoNegString,
							NdisParameterString);
	if(Status == NDIS_STATUS_SUCCESS)
	{
		if((USHORT) pConfigurationParameter->ParameterData.IntegerData == 0)
		{
			pAdapter->bAutoNeg = FALSE;
		}
		else
		{
			pAdapter->bAutoNeg = TRUE;
		}
	}

	if(pAdapter->bAutoNeg)
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:Config is AutoNeg Enabled\r\n")));
	else
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:Config is AutoNeg Disabled\r\n")));

	//See if user has defined new MAC address.
	NdisReadNetworkAddress(	&Status,
							(PVOID *) &pNewNetworkAddress,
							&nNewNetworkAddressLength,
							hConfiguration);
				
	if((Status == NDIS_STATUS_SUCCESS) && (nNewNetworkAddressLength != 0))
	{
		DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:Default MAC Address over-ride!\r\n")));
		if((nNewNetworkAddressLength != ETH_LENGTH_OF_ADDRESS)) 
		{
			DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:Invalid MAC address length!\r\n")));
		}
		else
		{
			pAdapter->MACAddress[0] = pNewNetworkAddress[0];
			pAdapter->MACAddress[1] = pNewNetworkAddress[1];
			pAdapter->MACAddress[2] = pNewNetworkAddress[2];
			pAdapter->MACAddress[3] = pNewNetworkAddress[3];
			pAdapter->MACAddress[4] = pNewNetworkAddress[4];
			pAdapter->MACAddress[5] = pNewNetworkAddress[5];

			DEBUGMSG(ZONE_INIT,(TEXT("Registry reads = %02X-%02X-%02X-%02X-%02X-%02X\r\n"),
				pNewNetworkAddress[0], 
				pNewNetworkAddress[1],
				pNewNetworkAddress[2],
				pNewNetworkAddress[3],
				pNewNetworkAddress[4],
				pNewNetworkAddress[5]));

			DEBUGMSG(ZONE_INIT,(TEXT("Adapter->MACAddress reads = %02X-%02X-%02X-%02X-%02X-%02X\r\n"),
				pAdapter->MACAddress[0],
				pAdapter->MACAddress[1],
				pAdapter->MACAddress[2],
				pAdapter->MACAddress[3],
				pAdapter->MACAddress[4],
				pAdapter->MACAddress[5]));
		}
	}
	else
	{
		Status = NDIS_STATUS_SUCCESS;
	}

	NdisCloseConfiguration(hConfiguration);

	if(Status != NDIS_STATUS_SUCCESS)
	{
			DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:ERROR: Specific Configuration Handler Failed!\r\n")));
			DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS:<== Configure Adapter\r\n")));
			return(Status);
	}
	
	DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS <== GetRegistrySettings  \r\n")));
	return(Status);
}

/*
 Function Name : 	BackOut				
 Description   :    
                    Called when the driver is unloaded. This releases all the OS resources
                     used by the chip and the driver.
 Parameters    :
                    MINIPORT_ADAPTER *Adapter
 Return Value  :
                    VOID
*/

void        BackOut				(MINIPORT_ADAPTER *pAdapter)
{
	DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS==> BackOut\r\n")));
	DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS==> Releasing all LPC3250 NDIS resources\r\n")));

	if (pAdapter->IsInterruptSet)
		NdisMDeregisterInterrupt(&pAdapter->InterruptInfo);

	if(pAdapter->pVAEMACBuffers != NULL)
	{
		UnMapRegisters (pAdapter->pVAEMACBuffers,pAdapter->dwEMACBuffersSize);
		pAdapter->pVAEMACBuffers = NULL;
	}

	if(pAdapter->pVALookAheadBuffer != NULL)
	{
		NdisFreeMemory(pAdapter->pVALookAheadBuffer,pAdapter->dwLookAheadBufferSize,0);
		pAdapter->pVALookAheadBuffer = NULL;
	}

	if(pAdapter->pTXPackets != NULL)
	{
		NdisFreeMemory(pAdapter->pTXPackets, sizeof(TX_PACKET) * pAdapter->dwTxStrides, 0);
		pAdapter->pTXPackets = NULL;
	}

	//Release the adapter structure.
    NdisFreeMemory(pAdapter, MINIPORT_ADAPTER_SIZE, 0);
	DEBUGMSG(ZONE_INIT, (TEXT("LPC3xxx NDIS<== BackOut\r\n")));
}
