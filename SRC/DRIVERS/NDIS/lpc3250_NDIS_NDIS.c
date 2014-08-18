/*
 *

 *Description:
 *      Contains the functions to be exported.
 *
 *
 */

#include <Ndis.h>
#include "lpc3250_NDIS_Adapter.h"
#include "lpc3250_NDIS_Proto.h"
//	P_ETHERNET_REGS_T p_ETH_REGS;
ETHERNET_REGS_T *p_ETH_REGS;

DBGPARAM dpCurSettings = {
  TEXT("LPC3250 NDIS"), {
    TEXT("Init"),TEXT("DeInit"),TEXT("Transmit"),TEXT("Receive"),
    TEXT("Interrupts"),TEXT(""),TEXT(""),TEXT(""),
    TEXT(""),TEXT(""),TEXT(""),TEXT(""),
    TEXT(""),TEXT("Info"),TEXT("Warnings"), TEXT("Errors")},
    MASK_TX | MASK_RX | MASK_WARN | MASK_ERROR | MASK_INFO
}; 

const NDIS_PHYSICAL_ADDRESS HighestAcceptedMax = NDIS_PHYSICAL_ADDRESS_CONST(-1,-1);

NTSTATUS    DriverEntry(IN PDRIVER_OBJECT  DriverObject, IN PUNICODE_STRING RegistryPath)
{
	NDIS_MINIPORT_CHARACTERISTICS 	Characteristics;
	NDIS_HANDLE						hNDISWrapper;
	NDIS_STATUS                     Status = NDIS_STATUS_SUCCESS;
	PHYSICAL_ADDRESS pa;

	DEBUGMSG(ZONE_INIT, (TEXT("\tNXP LPC3250 NDIS Windows Embedded CE 6.0 (NDIS%d.%d) Driver V1.0 Build#%d\r\n"), DRIVER_NDIS_MAJOR_VERSION, DRIVER_NDIS_MINOR_VERSION, DRIVER_BUILD_NUM));
	RETAILMSG(1, 
            (TEXT("NXP LPC3250 NDIS Windows Embedded CE 6.0 \r\n")));
	//Initialize the Wrapper.
	NdisMInitializeWrapper( &hNDISWrapper,
							DriverObject,
							RegistryPath,
							NULL
						);

    //Set up our Characteristics, but clear it first
    NdisZeroMemory((PVOID) &Characteristics, sizeof(Characteristics));

	//		Set the version of the supported NDIS layer (5.0)
	Characteristics.MajorNdisVersion        = DRIVER_NDIS_MAJOR_VERSION;
    Characteristics.MinorNdisVersion        = DRIVER_NDIS_MINOR_VERSION;

	//		Set the callback functions
	Characteristics.CheckForHangHandler     = LPC3xxx_NDIS_MiniportCheckforHang;									//Optional
	Characteristics.DisableInterruptHandler = LPC3xxx_NDIS_MiniportDisableInterrupt; 

	// Correct the bug of the interrupts
	// Removed the reference to MiniportEnableInterrupt from the miniport function table
	// and set up MiniportHandler to call MiniportEnableInterrupt itself.  
	// This minimizes the chance that an interrupt will go active after MiniportHandler 
	// but before interrupts are unmasked, again.
	Characteristics.EnableInterruptHandler  = NULL; 
	Characteristics.HaltHandler				= LPC3xxx_NDIS_MiniportHalt;				//Required
	Characteristics.HandleInterruptHandler  = LPC3xxx_NDIS_MiniPortHandleInterrupt;	//Required
    Characteristics.InitializeHandler		= LPC3xxx_MiniportInitialize;			//Required
	Characteristics.ISRHandler              = LPC3xxx_NDIS_MiniportISR;				//Required
	Characteristics.QueryInformationHandler = LPC3xxx_NDIS_MiniportQueryInformation;	//Required
	Characteristics.ReconfigureHandler      = NULL;
	Characteristics.ResetHandler            = LPC3xxx_NDIS_MiniportReset;				//Required
    Characteristics.SendHandler				= LPC3xxx_NDIS_MiniportSend;				//Required
	Characteristics.SetInformationHandler   = LPC3xxx_NDIS_MiniportSetInformation;
	Characteristics.TransferDataHandler     = NULL;

    //Register as an NDIS MiniPort Driver.
    Status = NdisMRegisterMiniport(	hNDISWrapper,
									&Characteristics,
									sizeof(NDIS_MINIPORT_CHARACTERISTICS));

    if(Status != NDIS_STATUS_SUCCESS)
    {
		NdisTerminateWrapper (hNDISWrapper, NULL);
		DEBUGMSG(ZONE_INIT | ZONE_ERROR, (TEXT("LPC3250 NDIS : NdisMRegisterMiniport Failed (%04x) !!\r\n"), Status));
    }

	DEBUGMSG(ZONE_INIT, (TEXT("LPC3250 NDIS <= DriverEntry\r\n")));

    return(Status);
}

BOOL __stdcall DllEntry	(
							HANDLE hDLL,
							DWORD dwReason,
							LPVOID lpReserved
						)
{
    switch (dwReason) 
	{
        case DLL_PROCESS_ATTACH:
				DEBUGREGISTER(hDLL);
				DEBUGMSG(ZONE_INFO, (TEXT("LPC3250 NDIS : DLL Process Attach\r\n")));
				break;
		case DLL_PROCESS_DETACH:
				DEBUGMSG(ZONE_INFO, (TEXT("LPC3250 NDIS :  DLL Process Detach\r\n")));
				break;
    }
    return TRUE;
}
