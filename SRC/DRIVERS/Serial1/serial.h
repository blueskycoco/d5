//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
//
// Use of this source code is subject to the terms of the Microsoft end-user
// license agreement (EULA) under which you licensed this SOFTWARE PRODUCT.
// If you did not accept the terms of the EULA, you are not authorized to use
// this source code. For a copy of the EULA, please see the LICENSE.RTF on your
// install media.
//
/*++
THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
PARTICULAR PURPOSE.

Module Name:  

ser16550.h

Abstract:

Definitions for serial 16550 library

Notes: 


--*/
#ifndef __SER16550_H__
#define __SER16550_H__
#include <ddkreg.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef ISR16550_INFO;
typedef XmitDataBuffer;
typedef RcvDataBuffer;

     // We use a callback for serial events
    typedef VOID		(*EVENT_FUNC)(PVOID Arg1, ULONG Arg2);

     // The library has a default baud table, but this can be replaced
    typedef struct	__PAIRS {
        ULONG   Key;
        ULONG   AssociatedValue;
    } PAIRS, *PPAIRS;

    typedef struct __LOOKUP_TBL {
        ULONG   Size;
        PPAIRS  Table;
    } LOOKUP_TBL, *PLOOKUP_TBL;

//
// @doc HWINTERNAL
// @struct LS_SERIAL_INFO | Passed to serial lib routines.
// 
    
    typedef struct __LS_SERIAL_INFO
    {
        volatile PULONG pRData;           //receive fifo register
        volatile PULONG  pTData;            //send fifo register
        volatile PULONG pLevel;        //rx&tx level register
        volatile PULONG pIir;            //interrupt status register
        volatile PULONG pCr;            //uart control register
        volatile PULONG pRcr;            //baud config resiger

        ULONG		RData;
        ULONG		TData;
        ULONG		Level;	
        ULONG		Iir;	
        ULONG		Cr;	
        ULONG		Rcr;	
        
         // We have an event callback into the MDD
        EVENT_FUNC EventCallback; // This callback exists in MDD
        PVOID        pMddHead;  // This is the first parm to callback
        
         // Keep a copy of DCB since we rely on may of its parms
        DCB         dcb;        // @field Device Control Block (copy of DCB in MDD)
         // And the same thing applies for CommTimeouts
        COMMTIMEOUTS CommTimeouts;  // @field Copy of CommTimeouts structure
        
         // Misc fields used by ser16550 library
        ULONG		OpenCount;	    // @field Count of simultaneous opens. 
//        PLOOKUP_TBL pBaudTable;     // @field Pointer to Baud Table
        ULONG		DroppedBytes;	// @field Number of dropped bytes 
        HANDLE		FlushDone;		// @field Handle to flush done event.
        BOOL		CTSFlowOff;		// @field Flag - CTS flow control state. 
        BOOL		DSRFlowOff;		// @field Flag - DSR flow control state. 
		BOOL		AddTXIntr;		// @field Flag - Fake a TX intr.
        COMSTAT		Status; 		// @field Bitfield representing Win32 comm status. 
        ULONG		CommErrors;		// @field Bitfield representing Win32 comm error status. 
        ULONG		ModemStatus;	// @field Bitfield representing Win32 modem status. 
        CRITICAL_SECTION	TransmitCritSec; // @field Protects UART Registers for non-atomic accesses
        CRITICAL_SECTION	RegCritSec; // @field Protects UART 
        ULONG		ChipID;			// @field Chip identifier (CHIP_ID_16550 or CHIP_ID_16450)        
        BOOL		PowerDown;      // did we power down the chip
        BOOL		bSuspendResume; // Indicate Suspend/Resume happens
        //
        // now hardware specific fields Duplicated Info
        DWORD       dwIrq;          // IRQ field
        DWORD       dwSysIntr;       // @field System Interrupt number for this peripheral
        PVOID       pVirtualStaticAddr;// Static Address.
        WCHAR       RegIsrDll[DEVDLL_LEN];// ISR Dll name.
        WCHAR       RegIsrHandler[DEVENTRY_LEN];//ISR Handle Name.

        volatile ISR16550_INFO * pIsrInfoVirt; 
        volatile XmitDataBuffer    * pXmitBuffer; 
        volatile RcvDataBuffer     * pReceiveBuffer; 
        HANDLE          hIsrHandler; 
        BOOL            bMoreXmitData;// For software xmit buffering
        DWORD           dwIsrIndex; // For test only
    } SER16550_INFO, *PSER16550_INFO;

// Values for SER16550_INFO.ChipID
#define CHIP_ID_16550	1
#define CHIP_ID_16450	2

/*
 * @doc HWINTERNAL
 * @struct SER_INFO | Private structure.
 */
    
     typedef struct __SER_INFO {
        SER16550_INFO  ser16550;

        // now hardware specific fields
        DWORD       dwSysIntr;       // @field System Interrupt number for this peripheral
        DWORD       dwDevIndex;     // @field Index of device

        PUCHAR      pBaseAddress;   // @field Start of serial registers - mapped

        UINT8       cOpenCount;     // @field Count of concurrent opens
        COMMPROP    CommProp;       // @field Pointer to CommProp structure.
        PVOID       pMddHead;       // @field First arg to mdd callbacks.
        BOOL        fIRMode;        // @field Boolean, are we running in IR mode?
    #ifdef EXAMINE_BOOTARGS
        PBOOT_ARGS  pBootArgs;      // @field Pointer to global boot args struct
    #endif    
        PHWOBJ      pHWObj;         // @field Pointer to PDDs HWObj structure
    } SER_INFO, *PSER_INFO;



// Here are the names of the values stored in the registry
#define PC_REG_DEVINDEX_VAL_NAME TEXT("DeviceArrayIndex") 
#define PC_REG_DEVINDEX_VAL_LEN  sizeof( DWORD )


    
    
// Here is the callback for serial events
    typedef VOID (*PFN_SER_EVENT) (
        PVOID pHandle,              // PHW_INDEP_INFO, but pdd doesn't know it
        UINT32 events               // What events where encountered?
        );

     // And now, all the function prototypes
BOOL SL_SetByteSize(PVOID pHead, ULONG ByteSize);
BOOL SL_SetStopBits(PVOID pHead, ULONG StopBits);
BOOL SL_SetParity(PVOID pHead, ULONG Parity);

     VOID SL_Init2(
        PVOID   pHead, //  points to device head
        PUCHAR  pRegBase, // Pointer to 16550 register base
        UINT8   RegStride, // Stride amongst the 16550 registers
        EVENT_FUNC EventCallback, // This callback exists in MDD
        PVOID   pMddHead,   // This is the first parm to callback
        PLOOKUP_TBL pBaudTable  // Pointer to baud rate table
        );
	PVOID SL_Init(
       ULONG   Identifier, // @parm Device identifier.
       PVOID   pMddHead,   // @parm First argument to mdd callbacks.
       PHWOBJ  pHWObj      // @parm Pointer to our own HW OBJ for this device
       );
    BOOL SL_PostInit(
        PVOID   pHead 
        );
    BOOL SL_Deinit(
        PVOID   pHead //  points to device head
        );
    BOOL SL_Open(
        PVOID   pHead 
        );
    ULONG SL_Close(
        PVOID   pHead
        );
    VOID SL_ClearDTR(
        PVOID   pHead 
        );
    VOID SL_SetDTR(
        PVOID   pHead 
        );
    VOID SL_ClearRTS(
        PVOID   pHead 
        );
    VOID SL_SetRTS(
        PVOID   pHead 
        );
    VOID SL_ClearBreak(
        PVOID   pHead 
        );
    VOID SL_SetBreak(
        PVOID   pHead 
        );
    VOID SL_ClearBreak(
        PVOID   pHead 
        );
    VOID SL_SetBreak(
        PVOID   pHead
        );
    ULONG SL_GetByteNumber(
        PVOID   pHead	     
        );
    VOID SL_DisableXmit(
        PVOID   pHead	
        );
    VOID SL_EnableXmit(
        PVOID   pHead	
        );
    BOOL SL_SetBaudRate(
        PVOID   pHead,
        ULONG   BaudRate	//      ULONG representing decimal baud rate.
        );
    BOOL SL_SetDCB(
        PVOID   pHead,	
        LPDCB   lpDCB       //     Pointer to DCB structure
        );
    ULONG SL_SetCommTimeouts(
        PVOID   pHead,	
        LPCOMMTIMEOUTS   lpCommTimeouts //  Pointer to CommTimeout structure
        );
    ULONG SL_GetRxBufferSize(
        PVOID pHead
        );
    PVOID SL_GetRxStart(
        PVOID   pHead
        );
    INTERRUPT_TYPE SL_GetInterruptType(
        PVOID pHead
        );
    ULONG SL_RxIntr(
        PVOID pHead,
        PUCHAR pRxBuffer,       // Pointer to receive buffer
        ULONG *pBufflen         //  In = max bytes to read, out = bytes read
        );
    ULONG SL_PutBytes(
        PVOID   pHead,
        PUCHAR  pSrc,	    // 	Pointer to bytes to be sent.
        ULONG   NumberOfBytes,  // 	Number of bytes to be sent.
        PULONG  pBytesSent	    // 	Pointer to actual number of bytes put.
        );
    VOID SL_TxIntr(
        PVOID pHead 
        );
    VOID SL_LineIntr(
        PVOID pHead
        );
    VOID SL_OtherIntr(
        PVOID pHead 
        );
    VOID SL_ModemIntr(
        PVOID pHead 
        );
    ULONG SL_GetStatus(
        PVOID	pHead,
        LPCOMSTAT	lpStat	// Pointer to LPCOMMSTAT to hold status.
        );
    VOID SL_Reset(
        PVOID   pHead
        );
    VOID SL_GetModemStatus(
        PVOID   pHead,
        PULONG  pModemStatus    //  PULONG passed in by user.
        );
    VOID SL_PurgeComm(
        PVOID   pHead,
        DWORD   fdwAction	    //  Action to take. 
        );
    BOOL SL_XmitComChar(
        PVOID   pHead,
        UCHAR   ComChar   //  Character to transmit. 
        );
    BOOL SL_PowerOn(
        PVOID   pHead
        );
    BOOL SL_PowerOff(
        PVOID   pHead
        );
    BOOL SL_Ioctl(
        PVOID pHead,
        DWORD dwCode,
        PBYTE pBufIn,
        DWORD dwLenIn,
        PBYTE pBufOut,
        DWORD dwLenOut,
        PDWORD pdwActualOut);

    VOID SL_TxIntrEx(
        PVOID pHead,
	    PUCHAR pTxBuffer,          // @parm Pointer to receive buffer
	    ULONG *pBufflen            // @parm In = max bytes to transmit, out = bytes transmitted
        );
    
#ifdef __cplusplus
}
#endif


#endif __SER16550_H__
