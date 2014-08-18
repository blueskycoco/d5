
#pragma once

#include <windows.h>
#include "wavedir.h"
#include "StreamOutput.h"

class StreamOutput
{
public:
	//********************************************************************
	// Initialization and de-init functions
	//********************************************************************

	// Constructors
	StreamOutput(class HardwareControl *pDeviceContext);

	// Destructor
	~StreamOutput();

	//********************************************************************
	// Stream open/close and info functions
	//********************************************************************

	// Open a stream for input or output
	DWORD stOpenStream(LPWAVEOPENDESC lpWOD,
		               DWORD dwFlags,
					   DWORD **StPtr);

	// Close an open stream
	DWORD stCloseStream(void);

	// Return pointer to current WAVE structure
	WAVEFORMATEX *stGetWaveFormat(void);

	// Return device context for class
	class HardwareControl *stGetDeviceContext(void);

	//********************************************************************
	// Stream buffer control functions
	//********************************************************************

	// Queue a buffer
	DWORD stQueueBuffer(LPWAVEHDR lpWaveHdr);

	// Get next WAVE buffer
	PBYTE stGetNextBuffer(void);

	// Return a buffer to the ACM
	void stReturnBuffer(LPWAVEHDR lpWaveHdr);

	// Break a playback loop
	DWORD stBreakLoop(void);

	// Return current byte playback count
	DWORD stGetByteCount(void);

	// Get data position
	DWORD stGetPos(PMMTIME pmmt);

	// Stop DMA
	DWORD stStop(void);

	// Start DMA
	DWORD stStart(void);

	// Reset stream
	DWORD stReset(void);

	// Output control thread
	void InterruptThread(void);

	volatile int m_buffsUsed;

private:
	// Instance counter and stream open/close counter
	static int m_instances;
	static int m_streamcnt;

	// Save device context associated with this class
	static class HardwareControl *m_pDeviceContext;

	// Audio thread handles, ID, and flags
	static HANDLE m_stThread;
	static DWORD m_stthreadID;
	static volatile DWORD m_stThreadStop;
	static volatile DWORD m_stThreadDone;

	// Audio event handle
	static HANDLE m_stEventDma;
	// Audio sysIntr values for DMA threads
	static DWORD m_stsysIntrDMA;
	// DMA driver handle
	static DWORD m_stdmaCtl;

	// Audio data attributes
	WAVEFORMATEX m_WaveFormat;

	// Used for callback
    HWAVE m_hWave;                   // handle for stream
    DRVCALLBACK* m_pfnCallback;      // client's callback
    DWORD m_dwInstance;              // client's instance data

	// Control and status data
    volatile BOOL m_bRunning;         // Is stream running or stopped
    LPWAVEHDR   m_lpWaveHdrHead;
    LPWAVEHDR   m_lpWaveHdrCurrent;
    LPWAVEHDR   m_lpWaveHdrTail;
    volatile PBYTE m_lpCurrData;            // position in current buffer
    volatile PBYTE m_lpCurrDataEnd;         // end of current buffer
    volatile DWORD m_dwByteCount;          // byte count since last reset
	DWORD       m_dwFlags;

    // Loopcount shouldn't really be here, since it's really for wave output only, but it makes things easier
    DWORD       m_dwLoopCount;          // Number of times left through loop

	// Buffer virtual and physical addresses, size in bytes
	PBYTE m_buffPhy [2];
	PBYTE m_buffVirt [2];
	DWORD m_buffSizeBytes;

	//********************************************************************
	// Misc functions
	//********************************************************************

	// Verifies a stram format is supported
	DWORD stIsSupportedFormat(LPWAVEFORMATEX pWaveFormat);

	// Open, close, and buffer return callback function
    void stDoDriverCallback(UINT msg,
							DWORD dwParam1,
							DWORD dwParam2);

	// Buffer return callback function
	void stDoCallbackReturnBuffer(LPWAVEHDR lpHdr);

	// Stream open callback function
	void stDoCallbackStreamOpened(void);

	// Stream close callback function
	void stDoCallbackStreamClosed(void);
};
