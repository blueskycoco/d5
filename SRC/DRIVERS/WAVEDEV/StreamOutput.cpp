
#include "wavemain.h"
#include "dma.h"
#include "dmadrv.h"

int StreamOutput::m_instances = 0;
int StreamOutput::m_streamcnt = 0;
class HardwareControl *StreamOutput::m_pDeviceContext = NULL;
HANDLE StreamOutput::m_stThread = INVALID_HANDLE_VALUE;
DWORD StreamOutput::m_stthreadID = 0;
volatile DWORD StreamOutput::m_stThreadStop = 0;
volatile DWORD StreamOutput::m_stThreadDone = 0;
HANDLE StreamOutput::m_stEventDma = NULL;
DWORD StreamOutput::m_stsysIntrDMA = SYSINTR_UNDEFINED;
DWORD StreamOutput::m_stdmaCtl = 0;

// Start thread
static void StartThread(class StreamOutput *pOutputStreamContext)
{
	pOutputStreamContext->InterruptThread();
}

//********************************************************************
// Initialization and de-init functions
//********************************************************************

// Constructors
StreamOutput::StreamOutput(class HardwareControl *pDeviceContext)
{
	DMASETUP_T dmaStp;
	DMAINFO_T *dmainfo;
	// Is this the first instance?
	m_instances++;
	if (m_instances == 1)
	{
		// Defaults
		m_pDeviceContext = NULL;
		m_stThread = INVALID_HANDLE_VALUE;
		m_stthreadID = 0;
		m_stThreadStop = 0;
		m_stThreadDone = 0;
		m_stEventDma = NULL;
		m_stsysIntrDMA = SYSINTR_UNDEFINED;
		m_stdmaCtl = 0;

		// Stream is locked out
		m_streamcnt = 1;

		// Save device context
		m_pDeviceContext = pDeviceContext;

		// Default flag states
		m_stThread = INVALID_HANDLE_VALUE;
		m_stthreadID = 0;
		m_stThreadStop = 0;
		m_stThreadDone = 0;

		// Create DMA output control thread
		m_stThread = CreateThread((LPSECURITY_ATTRIBUTES)NULL, 0,
			(LPTHREAD_START_ROUTINE) &StartThread,
			this, 0, &m_stthreadID);
		if (m_stThread == NULL)
		{
			RETAILMSG(1, 
				(TEXT("OutputStreamContext: Cannot initialize audio thread\r\n")));
			m_stThread = INVALID_HANDLE_VALUE;
			m_stThreadDone = 1;
			return;
		}

		// Setup DMA for the I2S1 TX channel
		dmaStp.dmaCh = DMAC_AUDIO_TX_CH;
		dmaStp.perID = DMA_PERID_I2S1_DMA1;
		dmaStp.SrcInc = 1;
		dmaStp.DestInc = 0;
		dmaStp.SrcWidth = DMAC_CHAN_SRC_WIDTH_16;
		dmaStp.DestWidth = DMAC_CHAN_DEST_WIDTH_16;
		dmaStp.SrcBurstSize = DMAC_CHAN_SRC_BURST_4;
		dmaStp.DestBurstSize = DMAC_CHAN_DEST_BURST_4;
		dmaStp.perFlowSource = DMAC_CHAN_FLOW_D_M2P;
		dmaStp.destPeripheral = DMAC_DEST_PERIP(DMA_PERID_I2S1_DMA1);
		dmaStp.srcPeripheral = 0;
		m_stdmaCtl = dmaAllocate(&dmaStp, (4 * 1024)); // Two 2K buffers
		RETAILMSG(1, 
				(TEXT("OutputStreamContext: dmaAllocate run ok\r\n")));
		if (m_stdmaCtl == 0)
		{
			// Couldn't setup DMA
			RETAILMSG(1, (TEXT("OutputStreamContext: Critical error setting up DMA\r\n")));
			m_stdmaCtl = 0;
			return;
		}

		// Create DMA event
		m_stEventDma = CreateEvent(NULL, FALSE, FALSE, NULL);
		if (m_stEventDma == NULL)
		{
			RETAILMSG(1, 
				(TEXT("OutputStreamContext: Failed to create DMA handler event.\r\n")));
			m_stEventDma = INVALID_HANDLE_VALUE;
			return;
		}

		// Get sysintr value for DMA channel interrupt
		dmainfo = dmaGetInfo(m_stdmaCtl);
	    if (!KernelIoControl(IOCTL_HAL_REQUEST_SYSINTR, &dmainfo->irq,
			sizeof(dmainfo->irq), &m_stsysIntrDMA, sizeof(m_stsysIntrDMA), NULL))
	    {
		    RETAILMSG(1, 
			    (TEXT("OutputStreamContext: Failed to request the DMA sysintr.\r\n")));
			m_stsysIntrDMA = SYSINTR_UNDEFINED;
			return;
	    }
	    RETAILMSG(1, 
		    (TEXT("OutputStreamContext: DMA sysintr is %d %d\r\n"), dmainfo->irq, m_stsysIntrDMA));

		// Save buffer data
		m_buffPhy [0] = (PBYTE) dmainfo->pBuffPhy;
		m_buffVirt [0] = (PBYTE) dmainfo->pBuffVirt;
		m_buffPhy [1] = m_buffPhy [0] + 2048;
		m_buffVirt [1] = m_buffVirt [0] + 2048;
		m_buffSizeBytes = dmainfo->buffSize;
		RETAILMSG(1, 
			(TEXT("OutputStreamContext: DMA buffer PHY 0x%x VIRT 0x%x Size %d\r\n"),
			m_buffPhy [0], m_buffVirt [0], m_buffSizeBytes));

		// Bind interrupt to event
		if (InterruptInitialize(m_stsysIntrDMA, m_stEventDma, NULL, 0) == FALSE)
		{
			// Cannot initialize interrupt
			RETAILMSG(1, 
				(TEXT("OutputStreamContext: Cannot initialize DMA interrupt\r\n")));
			return;
		}

		// Stream is free now
		m_streamcnt = 0;

		// Setup initial values
		m_lpWaveHdrHead = NULL;
		m_lpWaveHdrCurrent = NULL;
		m_lpWaveHdrTail = NULL;
		m_lpCurrData = NULL;
		m_lpCurrDataEnd = NULL;
		m_dwByteCount = 0;
		m_dwLoopCount = 0;
		m_bRunning = FALSE;

		m_buffsUsed = 0;

		// Setup an initial clocking rate for I2S and the CODEC
		m_pDeviceContext->hwAdjustSampleRate(16000, 16);
	}
}

// Destructor
StreamOutput::~StreamOutput()
{
	int to;
	m_instances--;
	if (m_instances == 0)
	{
		// Close event handle
		if (m_stEventDma == INVALID_HANDLE_VALUE)
		{
			// Stop thread if it is running
			m_stThreadStop = 1;
			to = 20;
			while ((to > 0) && (m_stThreadDone > 0))
			{
				SetEvent(m_stEventDma);
				Sleep(5);
				to--;
			}

			m_stThread = INVALID_HANDLE_VALUE;
			m_stthreadID = 0;

			// Close event handle
			CloseHandle(m_stEventDma);
			m_stEventDma = INVALID_HANDLE_VALUE;
		}

		// Disable interrupt and return sysIntr value
		if (m_stsysIntrDMA != SYSINTR_UNDEFINED)
		{
			InterruptDisable(m_stsysIntrDMA);
			KernelIoControl(IOCTL_HAL_RELEASE_SYSINTR, &m_stsysIntrDMA,
				sizeof(m_stsysIntrDMA), NULL, 0, NULL);
			m_stsysIntrDMA = SYSINTR_UNDEFINED;
		}

		// Return DMA buffer and close channel
		dmaDeAllocate(m_stdmaCtl);
		m_stdmaCtl = 0;

		m_pDeviceContext = NULL;

		// Safety
		m_streamcnt = 1;
	}
}

//********************************************************************
// Stream open/close and info functions
//********************************************************************

// Open a stream for input or output
DWORD StreamOutput::stOpenStream(LPWAVEOPENDESC lpWOD,
		                         DWORD dwFlags,
								 DWORD **StPtr)
{
	LPWAVEFORMATEX pWaveFormat = lpWOD->lpFormat;

	// Compression is not supported
	if ((dwFlags & WAVE_FORMAT_DIRECT) != 0)
	{
		return MMSYSERR_NOTSUPPORTED;
	}

	if (!stIsSupportedFormat(pWaveFormat))
    {
        return WAVERR_BADFORMAT;
    }

	// Exit now if this was a format check only
	if ((dwFlags & WAVE_FORMAT_QUERY) != 0)
	{
		// No new class is instanced here
		return MMSYSERR_NOERROR;
	}

	// Return 0 if the stream is already open
	if (m_streamcnt == 1)
	{
		return MMSYSERR_ERROR;
	}

	// Save copy of wave format
    memcpy(&m_WaveFormat, pWaveFormat, sizeof(WAVEFORMATEX));
    if (m_WaveFormat.wFormatTag == WAVE_FORMAT_PCM)
    {
		// Doesn't matter, but recommend by Microsoft
        m_WaveFormat.cbSize = 0;
    }
	m_dwFlags = dwFlags;

	// Save for callbacks
	m_pfnCallback = (DRVCALLBACK *)lpWOD->dwCallback;
	m_hWave = lpWOD->hWave;
	m_dwInstance = lpWOD->dwInstance;

	// Enable I2S clocks and power
	m_pDeviceContext->codecPowerUp(1);
	m_pDeviceContext->codecClockUp(1);
	m_pDeviceContext->I2SFlushFIFO();

	// Unpause channel
	m_pDeviceContext->I2SPause(0);

	// Setup I2S capabilities based on passed data
	if (m_WaveFormat.nChannels == 1)
	{
		m_pDeviceContext->I2SSetStereo(0);
	}
	else
	{
		m_pDeviceContext->I2SSetStereo(1);
	}

	// Setup clocking
	m_pDeviceContext->hwAdjustSampleRate(m_WaveFormat.nSamplesPerSec,
		m_WaveFormat.wBitsPerSample);

	// Clear DMA and enable DMA interrupts
	InterruptDone(m_stsysIntrDMA);

//    DEBUGMSG(1, (TEXT("  Channels      = %d\r\n"), m_WaveFormat.nChannels));
//    DEBUGMSG(1, (TEXT("  sampleRate    = %d\r\n"), m_WaveFormat.nSamplesPerSec));
//    DEBUGMSG(1, (TEXT("  bitsPerSample = %d\r\n"), m_WaveFormat.wBitsPerSample));
//    DEBUGMSG(1, (TEXT("  blockAlign    = %d\r\n"), m_WaveFormat.nBlockAlign));

	// Signal ACM that stream is opened
	stDoCallbackStreamOpened();
	*StPtr = (DWORD *) this;

	return MMSYSERR_NOERROR;
}

// Close an open stream
DWORD StreamOutput::stCloseStream(void)
{
	m_streamcnt--;

	stDoCallbackStreamClosed();

	return MMSYSERR_NOERROR;
}

// Return pointer to current WAVE structure
WAVEFORMATEX *StreamOutput::stGetWaveFormat(void)
{
	return &m_WaveFormat;
}

// Return device context for class
class HardwareControl *StreamOutput::stGetDeviceContext(void)
{
	return m_pDeviceContext;
}

//********************************************************************
// Stream buffer control functions
//********************************************************************

// Queue a buffer
DWORD StreamOutput::stQueueBuffer(LPWAVEHDR lpWaveHdr)
{
    if (!(lpWaveHdr->dwFlags & WHDR_PREPARED))
    {
        return WAVERR_UNPREPARED;
    }

    lpWaveHdr->dwFlags |= WHDR_INQUEUE;
    lpWaveHdr->dwFlags &= ~WHDR_DONE;
    lpWaveHdr->lpNext = NULL;
    lpWaveHdr->dwBytesRecorded = 0;

    if (!m_lpWaveHdrHead)
    {
        m_lpWaveHdrHead = lpWaveHdr;
    }
    else
    {
        m_lpWaveHdrTail->lpNext = lpWaveHdr;
    }

    m_lpWaveHdrTail = lpWaveHdr;

    // Note: Even if head & tail are valid, current may be NULL if we're in the middle of
    // a loop and ran out of data. So, we need to check specifically against current to
    // decide if we need to initialize it.
    if (!m_lpWaveHdrCurrent)
    {
        m_lpWaveHdrCurrent = lpWaveHdr;
        m_lpCurrData    = (PBYTE)lpWaveHdr->lpData;
        m_lpCurrDataEnd = (PBYTE)lpWaveHdr->lpData + lpWaveHdr->dwBufferLength;
        if (lpWaveHdr->dwFlags & WHDR_BEGINLOOP)    // if this is the start of a loop block
        {
            m_dwLoopCount = lpWaveHdr->dwLoops;     // save # of loops
        }
    }

    if (!m_bRunning)
    {
        stStart();
    }

    return MMSYSERR_NOERROR;
}

// Get next WAVE buffer
PBYTE StreamOutput::stGetNextBuffer(void)
{

    LPWAVEHDR lpOldHdr;
    LPWAVEHDR lpNewHdr;
    LPSTR pNewBuf = NULL;

    // Get a pointer to the current buffer which is now done being processed
    lpOldHdr = m_lpWaveHdrCurrent;

	if (!lpOldHdr)
    {
        return NULL;
    }

    // Are we in a loop
    // Note: a loopcount of 1 means we're not really in a loop
    if (m_dwLoopCount > 1)
    {
        // We're in a loop!
        if (lpOldHdr->dwFlags & WHDR_ENDLOOP)
        {
           // In loop, last buffer
            // If dwLoopCount was set to INFINITE, loop forever
            // (Note: this is not explicitly in the wave driver API spec)
            if (m_dwLoopCount != INFINITE)
            {
                m_dwLoopCount--;                    // decrement loop count
            }
           lpNewHdr = m_lpWaveHdrHead;           // go back to start of loop
        }
        else
        {
           // In loop, intermediate buffer
           lpNewHdr = lpOldHdr->lpNext;          // just go to next buffer in loop block
        }

        lpOldHdr = NULL;
    }
    else
    {
        // Not in a loop; return old buffer and get new buffer
        lpNewHdr = lpOldHdr->lpNext;

        m_lpWaveHdrHead = lpNewHdr;           // reset list head
        if (!lpNewHdr)
        {
            m_lpWaveHdrTail = NULL;             // no new buffer, reset tail to NULL
        }
        else if (lpNewHdr->dwFlags & WHDR_BEGINLOOP)    // if new buffer is start of a loop block
        {
            m_dwLoopCount = lpNewHdr->dwLoops;  // save # of loops
        }
    }

    m_lpWaveHdrCurrent = lpNewHdr;              // save current buffer pointer

    if (lpNewHdr)
    {
        m_lpCurrData    = (PBYTE)lpNewHdr->lpData;  // reinitialize data pointer
        m_lpCurrDataEnd = m_lpCurrData + lpNewHdr->dwBufferLength;
    }
    else
    {
        m_lpCurrData  = NULL;
        m_lpCurrDataEnd = NULL;
    }

    // Return the old buffer
    // This may cause the stream to be destroyed, so make sure that any calls to this function
    // are within an AddRef/Release block
    if (lpOldHdr)
    {
        stReturnBuffer(lpOldHdr);
    }

    return m_lpCurrData;
}

// Return a buffer to the ACM
void StreamOutput::stReturnBuffer(LPWAVEHDR lpWaveHdr)
{
	lpWaveHdr->dwFlags &= ~WHDR_INQUEUE;
	lpWaveHdr->dwFlags |= WHDR_DONE;
	stDoCallbackReturnBuffer(lpWaveHdr);
}

// Break a playback loop
DWORD StreamOutput::stBreakLoop(void)
{
    if (m_dwLoopCount > 0)
    {
        m_dwLoopCount = 0;

        LPWAVEHDR lpHdr;
        while (m_lpWaveHdrHead != m_lpWaveHdrCurrent)
        {
            lpHdr = m_lpWaveHdrHead;
            m_lpWaveHdrHead = lpHdr->lpNext;
            if (m_lpWaveHdrHead == NULL)
            {
                m_lpWaveHdrHead = NULL;
            }

			stReturnBuffer(lpHdr);
        }
    }

    return MMSYSERR_NOERROR;
}

// Return current byte playback count
DWORD StreamOutput::stGetByteCount(void)
{
	return m_dwByteCount;
}

// Get data position
DWORD StreamOutput::stGetPos(PMMTIME pmmt)
{
    switch (pmmt->wType)
    {

    case TIME_SAMPLES:
        pmmt->u.sample = (m_dwByteCount * 8) /
                         (m_WaveFormat.nChannels * m_WaveFormat.wBitsPerSample);
        break;

    case TIME_MS:
        if (m_WaveFormat.nAvgBytesPerSec != 0)
        {
            pmmt->u.ms = (m_dwByteCount * 1000) / m_WaveFormat.nAvgBytesPerSec;
            break;
        }
        // If we don't know avg bytes per sec, fall through to TIME_BYTES

    default:
        // Anything else, return TIME_BYTES instead.
        pmmt->wType = TIME_BYTES;

        // Fall through to TIME_BYTES
    case TIME_BYTES:
        pmmt->u.cb = m_dwByteCount;
    }

    return MMSYSERR_NOERROR;
}

// Stop DMA
DWORD StreamOutput::stStop(void)
{
    m_bRunning = FALSE;
    return MMSYSERR_NOERROR;
}

// Start DMA
DWORD StreamOutput::stStart(void)
{
    if (m_lpCurrData)
    {
		m_bRunning = TRUE;
        SetEvent(m_stEventDma);
    }

    return MMSYSERR_NOERROR;
}

// Reset stream
DWORD StreamOutput::stReset(void)
{
    // Stop stream for now.
    stStop();

    m_lpWaveHdrCurrent  = NULL;
    m_lpCurrData        = NULL;
    m_lpCurrDataEnd     = NULL;
    m_dwByteCount       = 0;
    m_dwLoopCount       = 0;

	m_buffsUsed = 0;

    LPWAVEHDR lpHdr;
    while (m_lpWaveHdrHead)
    {
        lpHdr = m_lpWaveHdrHead;
        m_lpWaveHdrHead = lpHdr->lpNext;
        if (m_lpWaveHdrHead == NULL)
        {
            m_lpWaveHdrTail = NULL;
        }
        stReturnBuffer(lpHdr);
    }

	dmaListReset(m_stdmaCtl);

    return MMSYSERR_NOERROR;
}

// Output control thread
void StreamOutput::InterruptThread(void)
{
	DWORD bytes, evid, queueroom;
	PBYTE pCurData;
	int nextfillidx;

	// TBD thread priority
	SetThreadPriority(m_stThread, THREAD_PRIORITY_HIGHEST); // TBD

	pCurData = NULL;
	nextfillidx = 0;
	ResetEvent(m_stEventDma);
	while (m_stThreadStop == 0)
	{
		// Wait for event
		evid = WaitForSingleObject(m_stEventDma, INFINITE);
		if (evid != WAIT_OBJECT_0)
		{
		    RETAILMSG(1, (TEXT("OutputStreamContext::InterruptThread event failure!!!!\r\n")));
			m_stThreadStop = 1;
		}
		if (m_stThreadStop == 0)
		{
			// Pop any previously completed buffers
			bytes = dmaCleanUp(m_stdmaCtl);
			while (bytes > 0)
			{
				m_dwByteCount += bytes;
				bytes = dmaCleanUp(m_stdmaCtl);
				m_buffsUsed--;
			}

			// Data fill DMA loop
			queueroom = 1;
			while ((queueroom == 1) && (m_buffsUsed < 2))
			{
				// Is there more data to play
				if (pCurData == NULL)
				{
					pCurData = stGetNextBuffer();
				}

				// Pop any previously completed buffers
				bytes = dmaCleanUp(m_stdmaCtl);
				while (bytes > 0)
				{
					m_dwByteCount += bytes;
					bytes = dmaCleanUp(m_stdmaCtl);
					m_buffsUsed--;
				}

				// Compute number of bytes to transfer
				bytes = (DWORD) (m_lpCurrDataEnd - pCurData);
				if (bytes > 2048)
				{
					bytes = 2048;
				}

				// Copy data to DMA buffer
			    memcpy(m_buffVirt [nextfillidx], pCurData, bytes);

				// Submit buffer to hardware
				if (dmaListEntry(m_stdmaCtl, (DWORD) m_buffPhy [nextfillidx],
					(I2S1_BASE + 8), bytes) == 0)
				{
					// Hardware cannot accept more yet
					queueroom = 0;
				}
				else
				{
					// Toggle buffer space
					nextfillidx = 1 - nextfillidx;
					m_buffsUsed++;

					pCurData += bytes;
					if (pCurData >= m_lpCurrDataEnd)
					{
						pCurData = NULL;
					}
				}
			}
		}

		if (dmaIsOn(m_stdmaCtl) == 0)
		{
			// Stalled, will need DMA restart later
			m_bRunning = FALSE;
		}

		// Re-enable DMA interrupts
		InterruptDone(m_stsysIntrDMA);
	}

	// Thread is exited
	m_stThreadDone = 1;
}

//********************************************************************
// Misc functions
//********************************************************************

// Returns supported status of a stream open
DWORD StreamOutput::stIsSupportedFormat(LPWAVEFORMATEX pWaveFormat)
{
	DWORD supported = 1;

	// Check supported audio formats
	if (pWaveFormat->wFormatTag != WAVE_FORMAT_PCM)
	{
		supported = 0;
	}

	// Check supported channels
	if ((pWaveFormat->nChannels != 1) && (pWaveFormat->nChannels != 2))
	{
		supported = 0;
	}

	// Check supported sample sizes
	if ((pWaveFormat->wBitsPerSample != 8) &&
		(pWaveFormat->wBitsPerSample != 16))
	{
		supported = 0;
	}

	return supported;
}

// Open, close, and buffer return callback function
void StreamOutput::stDoDriverCallback(UINT msg,
									  DWORD dwParam1,
									  DWORD dwParam2)
{
	m_pfnCallback(m_hWave, msg, m_dwInstance, dwParam1, dwParam2);
}

// Buffer return callback function
void StreamOutput::stDoCallbackReturnBuffer(LPWAVEHDR lpHdr)
{
	stDoDriverCallback(WOM_DONE, (DWORD)lpHdr, 0);
}

// Stream open callback function
void StreamOutput::stDoCallbackStreamOpened(void)
{
	stDoDriverCallback(WOM_OPEN, 0, 0);
}

// Stream close callback function
void StreamOutput::stDoCallbackStreamClosed(void)
{
	stDoDriverCallback(WOM_CLOSE, 0, 0);
}
