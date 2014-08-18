//*********************************************************************
//* Software that is described herein is for illustrative purposes only  
//* which provides customers with programming information regarding the  
//* products. This software is supplied "AS IS" without any warranties.  
//* NXP Semiconductors assumes no responsibility or liability for the 
//* use of the software, conveys no license or title under any patent, 
//* copyright, or mask work right to the product. NXP Semiconductors 
//* reserves the right to make changes in the software without 
//* notification. NXP Semiconductors also make no representation or 
//* warranty that such application will be suitable for the specified 
//* use without further testing or modification. 
//*
//* Copyright NXP Semiconductors
//*********************************************************************
//
// wavemain.cpp
//
// Audio driver API
//

#include "wavemain.h"

typedef struct
{
	class HardwareControl *pInContext;
	class HardwareControl *pOutContext;
	class StreamOutput    *pStreamOut;
} WM_DATA_T;
static WM_DATA_T wmaindat;

// -----------------------------------------------------------------------------
//
//  WAV_Init
//
//  Initialize wave device and driver
//
extern "C" DWORD WAV_Init(DWORD Index)
{

	DWORD devContext = (DWORD) &wmaindat;
	RETAILMSG(1, (TEXT("WAV_Init is begin!!!\r\n")));
	// The input and output device contexts are created here
	wmaindat.pOutContext = new HardwareControl(I2S_OUT_CH);
	 RETAILMSG(1, (TEXT("created the output device!!\r\n")));
	 wmaindat.pInContext = new HardwareControl(I2S_IN_CH);
	if ((wmaindat.pOutContext == NULL) ||
		(wmaindat.pInContext == 0))
	{
		RETAILMSG(1, (TEXT("WAV_Init: Class allocation failure\r\n")));
		devContext = 0;
	}
	else if ((wmaindat.pInContext->hwInitGood() == 0) ||
		(wmaindat.pOutContext->hwInitGood() == 0))
	{
		RETAILMSG(1, (TEXT("WAV_Init: HW Class init failure\r\n")));
		devContext = 0;
	}

	if (devContext != 0)
	{
		wmaindat.pStreamOut = new StreamOutput(wmaindat.pOutContext);
		if (wmaindat.pStreamOut == NULL)
		{
			RETAILMSG(1, (TEXT("WAV_Init: Stream Class init failure\r\n")));
			devContext = 0;
		}
	}

	return devContext;
}

// -----------------------------------------------------------------------------
//
//  WAV_Deinit
//
//  De-initialize wave device and driver
//
extern "C" BOOL WAV_Deinit(DWORD dwData)
{
	delete wmaindat.pOutContext;
	delete wmaindat.pInContext;
	delete wmaindat.pStreamOut;
	
	return TRUE;
}

// -----------------------------------------------------------------------------
//  WAV_Open
//
//  Open wave device
//
extern "C" PDWORD WAV_Open(DWORD dwData,
						   DWORD dwAccess,
						   DWORD dwShareMode)
{
    return (PDWORD) dwData;
}

// -----------------------------------------------------------------------------
//
//  WAV_Close
//
//  Close wave device
//
extern "C" BOOL WAV_Close(PDWORD pdwData)
{
    return TRUE;
}

void HandleWaveMessageInternal(PMMDRV_MESSAGE_PARAMS pParams, DWORD *pdwResult)
{
	PDWORD pTmp;

    //  set the error code to be no error first
    SetLastError(MMSYSERR_NOERROR);

    UINT uMsg = pParams->uMsg;
    UINT uDeviceId = pParams->uDeviceId;
    DWORD dwParam1 = pParams->dwParam1;
    DWORD dwParam2 = pParams->dwParam2;
//    class StreamOutput *pStreamOutput = (class StreamOutput *) pParams->dwUser;

    DWORD dwRet = MMSYSERR_NOERROR;

    switch (uMsg)
    {
    case WODM_GETVOLUME:
        pTmp = (PDWORD) dwParam1;
        *pTmp = (DWORD) wmaindat.pOutContext->codecGetMasterVolume();
        break;

    case WODM_SETVOLUME:
        if (wmaindat.pOutContext->codecSetMasterVolume(dwParam1) == 0)
		{
			dwRet = MMSYSERR_ERROR;
		}
        break;

	case WODM_GETNUMDEVS:
        dwRet = 1;
        break;

    case WIDM_GETNUMDEVS:
        dwRet = 0;
        break;

    case WODM_GETDEVCAPS:
		dwRet = (DWORD) wmaindat.pOutContext->hwWaveDevGetCaps((PVOID )dwParam1,
			dwParam2);
        break;

    case WIDM_GETDEVCAPS:
		dwRet = (DWORD) wmaindat.pInContext->hwWaveDevGetCaps((PVOID )dwParam1,
			dwParam2);
        break;

    case WODM_PAUSE:
		if (wmaindat.pOutContext->I2SPause(TRUE) == 0)
		{
			dwRet = MMSYSERR_ERROR;
		}
        break;

	case WIDM_STOP:
		if (wmaindat.pInContext->I2SPause(TRUE) == 0)
		{
			dwRet = MMSYSERR_ERROR;
		}
        break;

    case WODM_RESTART:
		if (wmaindat.pOutContext->I2SPause(FALSE) == 0)
		{
			dwRet = MMSYSERR_ERROR;
		}
        break;

	case WIDM_START:
		if (wmaindat.pInContext->I2SPause(FALSE) == 0)
		{
			dwRet = MMSYSERR_ERROR;
		}
        break;

    case WODM_OPEN:
		dwRet = wmaindat.pStreamOut->stOpenStream(
			(LPWAVEOPENDESC) dwParam1, dwParam2,
			(DWORD **) pParams->dwUser);
		break;

    case WIDM_OPEN:
		dwRet = MMSYSERR_ERROR;
		break;

    case WODM_CLOSE:
		dwRet = wmaindat.pStreamOut->stCloseStream();
		break;

	case WIDM_CLOSE:
		dwRet = MMSYSERR_ERROR;
		break;

	case WODM_WRITE:
		dwRet = wmaindat.pStreamOut->stQueueBuffer((LPWAVEHDR) dwParam1);
		break;

	case WIDM_ADDBUFFER:
		dwRet = MMSYSERR_ERROR;
		break;

	case WODM_GETPOS:
		dwRet = wmaindat.pStreamOut->stGetPos((LPMMTIME) dwParam1);
		break;

	case WIDM_GETPOS:
		dwRet = MMSYSERR_ERROR;
		break;

	case WODM_BREAKLOOP:
		dwRet = wmaindat.pStreamOut->stBreakLoop();
		break;

	case WODM_RESET:
		dwRet = wmaindat.pStreamOut->stReset();
		break;

	case WIDM_RESET:
		dwRet = MMSYSERR_ERROR;
		break;

    case WODM_PREPARE:
    case WODM_UNPREPARE:
	case WIDM_PREPARE:
	case WIDM_UNPREPARE:
		// These can be implemented for better DMA support, but require
		// a lot more software overhead in the audio drivers

	case WIDM_SETPROP:
	case WODM_GETPROP:
	case WODM_SETPROP:
	case WIDM_GETPROP:
    case WODM_GETEXTDEVCAPS:
		// Not supported;

	case WODM_GETPLAYBACKRATE:
    case WODM_SETPLAYBACKRATE:
    case WODM_SETPITCH:
    case WODM_GETPITCH:
		// Optional, not supported

    default:
        dwRet  = MMSYSERR_NOTSUPPORTED;
    }

    // Pass the return code back via pdwResult
    if (pdwResult)
    {
        *pdwResult = dwRet;
    }
}

// -----------------------------------------------------------------------------
//
//  HandleWaveMessage
//
//  Handle messages from IOCTL
//
BOOL HandleWaveMessage(PMMDRV_MESSAGE_PARAMS pParams, DWORD *pdwResult)
{
    _try {
        HandleWaveMessageInternal(pParams, pdwResult);
    } _except(EXCEPTION_EXECUTE_HANDLER) {
        SetLastError(E_FAIL);
    }

    return TRUE;
}

// -----------------------------------------------------------------------------
//
//  WAV_IOControl
//
//  Close wave device
//
extern "C" BOOL WAV_IOControl(PDWORD pdwOpenData,
							  DWORD dwCode,
							  PBYTE pBufIn,
							  DWORD dwLenIn,
							  PBYTE pBufOut,
							  DWORD dwLenOut,
							  PDWORD pdwActualOut)
{
	DWORD dwResult = MMSYSERR_NOERROR;

	_try
    {
        switch (dwCode)
        {
        case IOCTL_WAV_MESSAGE:
            return HandleWaveMessage((PMMDRV_MESSAGE_PARAMS)pBufIn, (DWORD *)pBufOut);

        // Power management functions.
        case IOCTL_POWER_CAPABILITIES:
            if ((pBufOut == NULL) || (dwLenOut < sizeof(POWER_CAPABILITIES)))
			{
                dwResult = MMSYSERR_INVALPARAM;
			}
            else
            {
                wmaindat.pOutContext->GetPowerCapabilities(pBufOut);
                *pdwActualOut = sizeof(POWER_CAPABILITIES);
            }
            break;
		
		case IOCTL_POWER_QUERY:
            if ((pBufOut == NULL) || (dwLenOut < sizeof(CEDEVICE_POWER_STATE)))
			{
                dwResult = MMSYSERR_INVALPARAM;
			}
            else
            {
                PCEDEVICE_POWER_STATE pState = (PCEDEVICE_POWER_STATE) pBufOut;

                // Check if hardware handles this power state.
                if (!wmaindat.pOutContext->QueryPowerState(*pState))
				{
                    dwResult = MMSYSERR_INVALPARAM;
				}
            }
            break;

		case IOCTL_POWER_SET:
            if ((pBufOut == NULL) || (dwLenOut < sizeof(CEDEVICE_POWER_STATE)))
			{
                dwResult = MMSYSERR_INVALPARAM;
			}
            else
            {
                PCEDEVICE_POWER_STATE pState = (PCEDEVICE_POWER_STATE) pBufOut;

                dwResult = wmaindat.pOutContext->SetPowerState(*pState);

                if (dwResult == MMSYSERR_NOERROR)
                {
                    *pState = wmaindat.pOutContext->GetPowerState();
                    *pdwActualOut = sizeof(CEDEVICE_POWER_STATE);
                }
                else
				{
                    dwResult = MMSYSERR_INVALPARAM;
				}
            }
            break;

		case IOCTL_POWER_GET:
            if ((pBufOut == NULL) || (dwLenOut < sizeof(CEDEVICE_POWER_STATE)))
			{
                dwResult = MMSYSERR_INVALPARAM;
			}
            else
            {
                *((PCEDEVICE_POWER_STATE) pBufOut) =
					wmaindat.pOutContext->GetPowerState();
                *pdwActualOut = sizeof(CEDEVICE_POWER_STATE);
            }
            break;
		}
    }

	_except (EXCEPTION_EXECUTE_HANDLER)
    {
        RETAILMSG(1, (TEXT("EXCEPTION IN WAV_IOControl!!!!\r\n")));
        SetLastError(E_FAIL);
    }

    if (dwResult != MMSYSERR_NOERROR)
    {
        SetLastError(dwResult);
        return FALSE;
    }

    return TRUE;
}

// -----------------------------------------------------------------------------
//
//  DllMain
//
//  DLL entry point
//
BOOL CALLBACK DllMain(HANDLE hDLL,
                      DWORD dwReason,
                      LPVOID lpvReserved)
{
    switch (dwReason) {
        case DLL_PROCESS_ATTACH :
            DisableThreadLibraryCalls((HMODULE) hDLL);
            break;

        case DLL_PROCESS_DETACH :
            break;

        case DLL_THREAD_DETACH :
            break;

        case DLL_THREAD_ATTACH :
            break;

        default :
            break;
    }
    return TRUE;
}