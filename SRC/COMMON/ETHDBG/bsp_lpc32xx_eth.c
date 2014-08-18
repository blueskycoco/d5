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
// bsp_lpc32xx_eth.h
//
// Ethernet support functions for KITL and EBOOT download
//

#include <windows.h>
#include <halether.h>
#include <oal.h>
#include "image_cfg.h"
#include "bsp_phyc32xx_phy.h"

#include "lpc32xx_mac.h"
#include "lpc32xx_clkpwr.h"
#include "phy3250_board.h"

// Driver data
static ETHERNET_REGS_T *pEregs;
static CLKPWR_REGS_T *pClkpwr;
static BOOL init;
static UINT32 smac [3];
static UINT32 g_dmabase;
static UINT32 gdma_size;
static TXRX_DESC_T *pTXDesc;
static UINT32 *pTXStatus;
static UINT32 pTXVBuffs [ENET_MAX_TX_PACKETS];
static TXRX_DESC_T *pRXDesc;
static RX_STATUS_T *pRXStatus;
static UINT32 pRXVBuffs [ENET_MAX_RX_PACKETS];

// External PHYTEC hardware structure with MAC address
extern PHY_HW_T phyhwdesc;

// Returns the ethernet DMA buffer address and size
extern void OEMGetEthBuffers(UINT32 *dmabase, UINT32 *dmasize);

UINT16 LPC32XX_Eth_SendFrame(UINT8 *pbData, UINT32 length);

//------------------------------------------------------------------------------
//
// msDelay
//
// Delay a number of milliSeconds
//
VOID msDelay(UINT32 ms)
{
	UINT32 mswval = OALGetTickCount() + ms;
	while (mswval > OALGetTickCount());
}

//------------------------------------------------------------------------------
//
// RMII_Write
//
// Write a value to a RMII PHY register
//
static BOOL RMII_Write (UINT32 PhyReg, UINT32 Value)
{
	UINT32 mst = 250;
	BOOL sts = FALSE;

	// Write value at PHY address and register
	pEregs->madr = (PHYDEF_PHYADDR << 5) | PhyReg;
	pEregs->mwtd = Value;

	// Wait for unbusy status
	while (mst > 0)
	{
		if ((pEregs->mind & MIND_BUSY) == 0)
		{
			mst = 0;
			sts = TRUE;
		}
		else
		{
			mst--;
			msDelay(1);
		}
    }

	if (sts == FALSE)
	{
		OALMSGS((OAL_ETHER && OAL_FUNC),
			(L"RMII_Write:PHY register %d write timed out = 0x%08x\r\n", PhyReg));
	}

	return sts;
}

//------------------------------------------------------------------------------
//
// RMII_Read
//
// Read a value from a RMII PHY register
//
BOOL RMII_Read(UINT32 PhyReg, UINT32 *data) 
{
	UINT32 mst = 250;
	BOOL sts = FALSE;

	// Read value at PHY address and register
	pEregs->madr = (PHYDEF_PHYADDR << 5) | PhyReg;
	pEregs->mcmd = MCMD_READ;

	// Wait for unbusy status
	while (mst > 0)
	{
		if ((pEregs->mind & MIND_BUSY) == 0)
		{
			mst = 0;
			*data = pEregs->mrdd;
			sts = TRUE;
		}
		else
		{
			mst--;
			msDelay(1);
		}
    }

	pEregs->mcmd = 0;

	if (sts == FALSE)
	{
		OALMSGS((OAL_ETHER && OAL_FUNC),
			(L"RMII_Read:PHY register %d read timed out = 0x%08x\r\n", PhyReg));
	}

	return sts;
}

//------------------------------------------------------------------------------
//
// HYPHYReset
//
// Reset the PHY
//
BOOL HYPHYReset(VOID)
{
	BOOL goodacc;
	UINT32 tmp1, mst;

	// Reset the PHY and wait for reset to complete
	goodacc = RMII_Write(PHY_REG_BMCR, PHY_BMCR_RESET_BIT);
	if (goodacc == FALSE)
	{
		return FALSE;
	}
	mst = 400;
	goodacc = FALSE;
	while (mst > 0)
	{
		RMII_Read(PHY_REG_BMCR, &tmp1);
		if ((tmp1 & PHY_BMCR_RESET_BIT) == 0)
		{
			mst = 0;
			goodacc = TRUE;
		}
		else
		{
			mst--;
			msDelay(1);
		}
	}

	return goodacc;
}

//------------------------------------------------------------------------------
//
// txrx_setup
//
// Setup transmit and receive descriptors
//
BOOL txrx_setup(VOID)
{
	INT32 idx;
	UINT32 *pTXStatusL, pbase1, pbase2, pbase3;
	TXRX_DESC_T *pTXRXDesc;
	RX_STATUS_T *pRXStatusL;

	// Get physical address and size of DMA buffers
	OEMGetEthBuffers(&g_dmabase, &gdma_size);

	// Setup base pointers
	pbase1 = g_dmabase;     // Start of descriptors
	pbase2 = pbase1 + 256;  // Start of statuses
	pbase3 = pbase1 + 1024; // Start of buffers

	OALMSGS((OAL_ETHER && OAL_FUNC),
		(L"+txrx_setup buffer physical address at 0x%x with size %d\r\n",
        g_dmabase, gdma_size));

	// Setup pointers to TX structures
	pEregs->txdescriptor =  pbase1;
	pEregs->txstatus = pbase2;
	pEregs->txdescriptornumber = (ENET_MAX_TX_PACKETS - 1);

	// Save base address of TX descriptor table and TX status
	pTXRXDesc = (TXRX_DESC_T *) OALPAtoVA(pbase1, FALSE);
	pTXStatusL = (UINT32 *) OALPAtoVA(pbase2, FALSE);
	pTXDesc = pTXRXDesc;
	pTXStatus = pTXStatusL;

	// Build TX descriptors
	for (idx = 0; idx < ENET_MAX_TX_PACKETS; idx++)
	{
		pTXRXDesc->packet = pbase3;
		pTXRXDesc->control = 0;
		*pTXStatusL = 0;

		// Save virtual address of buffer
		pTXVBuffs [idx] = (UINT32) OALPAtoVA(pbase3, FALSE);

		// Next descriptor and status
		pTXRXDesc++;
		pTXStatusL++;
		pbase1 += sizeof (TXRX_DESC_T);
		pbase2 += sizeof (UINT32);
		pbase3 += ENET_MAXF_SIZE;
	}

	// Setup pointers to RX structures
	pEregs->rxdescriptor = pbase1;
	pEregs->rxstatus = pbase2;
	pEregs->rxdescriptornumber = (ENET_MAX_RX_PACKETS - 1);

	// Save base address of RX descriptor table and RX status
	pRXDesc = pTXRXDesc;
	pRXStatus = pRXStatusL = (RX_STATUS_T *) pTXStatusL;

	// Build RX descriptors
	for (idx = 0; idx < ENET_MAX_TX_PACKETS; idx++)
	{
		pTXRXDesc->packet = pbase3;
		pTXRXDesc->control = 0x80000000 | (ENET_MAXF_SIZE - 1);
		pRXStatusL->statusinfo = 0;
		pRXStatusL->statushashcrc = 0;

		// Save virtual address of buffer
		pRXVBuffs [idx] = (UINT32) OALPAtoVA(pbase3, FALSE);

		// Next descriptor and status
		pTXRXDesc++;
		pRXStatusL++;
		pbase1 += sizeof (TXRX_DESC_T);
		pbase2 += sizeof (UINT32);
		pbase3 += ENET_MAXF_SIZE;
	}

    return TRUE;
}

//------------------------------------------------------------------------------
//
// HWInit
//
// Initialize ethernet and PHY hardware
//
BOOL HWInit(VOID)
{
	BOOL btemp, goodacc;
	UINT32 tmp1, tmp2, mst = 250;

	if (init == TRUE)
	{
		// Previously initialized, do nothing
		return TRUE;
	}
	init = TRUE;

	// Enable MAC interface
	pClkpwr->clkpwr_macclk_ctrl = (CLKPWR_MACCTRL_HRCCLK_EN |
		CLKPWR_MACCTRL_MMIOCLK_EN | CLKPWR_MACCTRL_DMACLK_EN |
#ifdef USE_PHY_RMII
		CLKPWR_MACCTRL_USE_RMII_PINS);
#else
		CLKPWR_MACCTRL_USE_MII_PINS);
#endif

	// Set RMII management clock rate. This clock should be slower
	// than 12.5MHz (for NXP PHYs only). For a divider of 28, the
	// clock rate when HCLK is 150MHz will be 5.4MHz
	pEregs->mcfg = MCFG_CLOCK_SELECT(MCFG_CLOCK_HOST_DIV_28);

	// Reset all MAC logic
	pEregs->mac1 = (MAC1_SOFT_RESET | MAC1_SIMULATION_RESET |
		MAC1_RESET_MCS_TX | MAC1_RESET_TX | MAC1_RESET_MCS_RX |
		MAC1_RESET_RX);
	pEregs->command = (COMMAND_REG_RESET | COMMAND_TXRESET |
		COMMAND_RXRESET);
	msDelay(10);

	// Initial MAC initialization
	pEregs->mac1 = MAC1_PASS_ALL_RX_FRAMES;
	pEregs->mac2 = (MAC2_PAD_CRC_ENABLE | MAC2_CRC_ENABLE);
	pEregs->maxf = ENET_MAXF_SIZE;

	// Maximum number of retries, 0x37 collision window, gap */
	pEregs->clrt = (CLRT_LOAD_RETRY_MAX(0xF) |
		CLRT_LOAD_COLLISION_WINDOW(0x37));
	pEregs->ipgr = IPGR_LOAD_PART2(0x12);

#ifdef USE_PHY_RMII
	// RMII setup
	pEregs->command = (COMMAND_RMII | COMMAND_PASSRUNTFRAME);
	pEregs->supp = SUPP_RESET_RMII;
	msDelay(10);
#else
	// MII setup
	pEregs->command = COMMAND_PASSRUNTFRAME;
#endif

	// Reset PHY
	goodacc = HYPHYReset();
	if (goodacc == FALSE)
	{
		OALMSGS((OAL_ETHER && OAL_FUNC),
			(L"ENET:Reset of PHY timed out\r\n"));
		return FALSE;
	}

	// Get PHY ID
	RMII_Read(PHY_REG_IDR1, &tmp1);
	RMII_Read(PHY_REG_IDR2, &tmp2);
	OALMSGS((OAL_ETHER && OAL_FUNC),
		(L"ENET:PHY ID 1 register = 0x%08x\r\n", tmp1));
	OALMSGS((OAL_ETHER && OAL_FUNC),
		(L"ENET:PHY ID 2 register = 0x%08x\r\n", tmp2));

	// Enable rate auto-negotiation for the link
    if (RMII_Write(PHY_REG_BMCR,
		(PHY_BMCR_SPEED_BIT | PHY_BMCR_AUTON_BIT)) == FALSE)
	{
		return FALSE;
	}

	// Wait up to 5 seconds for auto-negotiation to finish
	mst = 5000;
	goodacc = TRUE;
	btemp = FALSE;
	while (mst > 0)
	{
		goodacc &= RMII_Read(PHY_REG_BMSR, &tmp1);
		if ((tmp1 & PHY_BMSR_AUTON_COMPLETE) != 0)
		{
			mst = 0;
			btemp = TRUE;
			OALMSGS((OAL_ETHER && OAL_FUNC),
				(L"ENET:auto-negotiation complete\r\n"));
		}
		else
		{
			mst--;
			msDelay(1);
		}
	}
	if ((goodacc == FALSE) || (btemp == FALSE))
	{
		OALMSGS((OAL_ETHER && OAL_FUNC),
			(L"ENET:auto-negotiation failed\r\n"));
		return FALSE;
	}

	// Check link status
	mst = 1000;
	goodacc = TRUE;
	btemp = FALSE;
	while (mst > 0)
	{
		goodacc &= RMII_Read(PHY_REG_BMSR, &tmp1);
		if ((tmp1 & PHY_BMSR_LINKUP_STATUS) != 0)
		{
			mst = 0;
			btemp = TRUE;
			OALMSGS((OAL_ETHER && OAL_FUNC),
				(L"ENET:Link status up\r\n"));
		}
		else
		{
			mst--;
			msDelay(1);
		}
	}
	if ((goodacc == FALSE) || (btemp == FALSE))
	{
		OALMSGS((OAL_ETHER && OAL_FUNC),
			(L"ENET:Link status failure\r\n"));
		return FALSE;
	}

	// Try 100MBase/full duplex
	goodacc = btemp = FALSE;
	if ((tmp1 & PHY_BMSR_TX_FULL) != 0)
	{
		// Setup for full duplex and 100MBase
		goodacc = btemp = TRUE;
	}
	else if ((tmp1 & PHY_BMSR_TX_HALF) != 0)
	{
		// Setup for half duplex and 100MBase
		goodacc = TRUE;
	}
	else if ((tmp1 & PHY_BMSR_TX_HALF) != 0)
	{
		// Setup for full duplex and 10MBase
		btemp = TRUE;
	}

	// Configure Full/Half Duplex mode
	if (btemp == TRUE)
	{
		// 10MBase full duplex is supported
		pEregs->mac2 |= MAC2_FULL_DUPLEX;
		pEregs->command |= COMMAND_FULLDUPLEX;
		pEregs->ipgt = IPGT_LOAD(0x15);
		OALMSGS((OAL_ETHER && OAL_FUNC), (L"ENET:FULL DUPLEX\r\n"));
	}
	else
	{
		pEregs->ipgt = IPGT_LOAD(0x12);
		OALMSGS((OAL_ETHER && OAL_FUNC), (L"ENET:HALF DUPLEX\r\n"));
	}

	// Configure 100MBit/10MBit mode
	if (goodacc == TRUE)
	{
		// 100MBase mode
		pEregs->supp = SUPP_SPEED;
		OALMSGS((OAL_ETHER && OAL_FUNC), (L"ENET:100MBase\r\n"));
	}
	else
	{
		// 10MBase mode
		pEregs->supp = 0;
		OALMSGS((OAL_ETHER && OAL_FUNC), (L"ENET:10MBase\r\n"));
	}

	// Save station address
	pEregs->sa [2] = smac [0];
	pEregs->sa [1] = smac [1];
	pEregs->sa [0] = smac [2];

	// Setup TX and RX descriptors
	txrx_setup();

	// Enable broadcast and matching address packets
	pEregs->rxfliterctrl = (RXFLTRW_ACCEPTUBROADCAST |
		RXFLTRW_ACCEPTPERFECT);

//	pEregs->txproduceindex = 0;
//	pEregs->rxconsumeindex = 0;

	// Clear and enable interrupts
	pEregs->intclear = 0xFFFF;
	pEregs->intenable = 0;

	// Enable receive and transmit mode of MAC ethernet core
	pEregs->command |= (COMMAND_RXENABLE | COMMAND_TXENABLE);
	pEregs->mac1 |= MAC1_RECV_ENABLE;

	// Perform a 'dummy' send of the first ethernet frame with a size of 0
	// to 'prime' the MAC. The first packet after a reset seems to wait
	// until at least 2 packets are ready to go.
//	goodacc = 0;
//	LPC32XX_Eth_SendFrame((UINT8 *) &goodacc, 4);

	return TRUE;
}

//------------------------------------------------------------------------------
//
// HWDeInit
//
// De-initialize ethernet and PHY hardware
//
BOOL HWDeInit(VOID)
{
	if (init == TRUE)
	{
		init = FALSE;

		// Reset PHY
		(void) HYPHYReset();

		// Reset all MAC logic
		pEregs->mac1 = (MAC1_SOFT_RESET | MAC1_SIMULATION_RESET |
			MAC1_RESET_MCS_TX | MAC1_RESET_TX | MAC1_RESET_MCS_RX |
			MAC1_RESET_RX);
		pEregs->command = (COMMAND_REG_RESET | COMMAND_TXRESET |
			COMMAND_RXRESET);
		msDelay(2);

		// Disable MAC clocks, but keep MAC interface active
#ifdef USE_PHY_RMII
		pClkpwr->clkpwr_macclk_ctrl = CLKPWR_MACCTRL_USE_RMII_PINS;
#else
		pClkpwr->clkpwr_macclk_ctrl = CLKPWR_MACCTRL_USE_MII_PINS;
#endif
	}

	return TRUE;
}

//------------------------------------------------------------------------------
//
// LPC32XX_Eth_Init
//
// Initialize ethernet interface for KITL/download support
//
BOOL LPC32XX_Eth_Init(UINT8 *pAddress, UINT32 offset, UINT16 mac[3])
{
    BOOL rc = FALSE;

	pClkpwr = (CLKPWR_REGS_T *) OALPAtoVA((UINT32) CLKPWR, FALSE);
	pEregs = (ETHERNET_REGS_T *) OALPAtoVA((UINT32) pAddress, FALSE);

	// Set MAC address from hardware
	mac [0] = (phyhwdesc.mac[0] | (phyhwdesc.mac[1] << 8));
	mac [1] = (phyhwdesc.mac[2] | (phyhwdesc.mac[3] << 8));
	mac [2] = (phyhwdesc.mac[4] | (phyhwdesc.mac[5] << 8));

	// Save MAC address
	smac [0] = (UINT32) (mac[0]);
	smac [1] = (UINT32) (mac[1]);
	smac [2] = (UINT32) (mac[2]);

	OALMSGS((OAL_ETHER && OAL_FUNC), (
        L"+LPC32XX_Eth_Init(0x%08x, 0x%08x, %02x:%02x:%02x:%02x:%02x:%02x)\r\n",
        pAddress, offset, mac[0]&0xFF, mac[0]>>8, mac[1]&0xFF, mac[1]>>8,
        mac[2]&0xFF, mac[2]>>8
    ));

	rc = HWInit();

	// De-init if an error occurred
	init = FALSE;
	if (rc == FALSE)
	{
		HWDeInit();
	}

    return rc;
}

#if 0
//------------------------------------------------------------------------------
//
// LPC32xx_Eth_InitDMABuffer
//
// Initialize DMA buffers
//
BOOL   LPC32xx_Eth_InitDMABuffer(UINT32 address, UINT32 size)
{
	UINT32 tmp;

	// Align on 16 byte boundary and save base address and size for DMA location
	g_dmabase = address & ~0xF;
	g_dmabase += 0x10;
	gdma_size = size - (g_dmabase - address);
    OALMSGS((OAL_ETHER && OAL_FUNC), (
	    L"+LPC32xx_Eth_InitDMABuffer 0x%x 0x%x\r\n", address, size));

	// Verify that enough space is allocated for the DMA buffers to store
	// 4 TX buffers, 4 RX buffers, and descriptors and statuses for the
	// buffers. 256 bytes are used for descriptors/statuses.
	tmp = 512 + (ENET_MAXF_SIZE * (ENET_MAX_TX_PACKETS + ENET_MAX_RX_PACKETS));
	if (tmp > gdma_size)
	{
		// Not enough space
	    OALMSGS((OAL_ETHER && OAL_FUNC), (
		    L"+LPC32xx_Eth_InitDMABuffer-not enough space for DMA buffers\r\n"));
		return FALSE;
	}

	return TRUE;
}
#endif

//------------------------------------------------------------------------------
//
// LPC32XX_Eth_Deinit
//
// De-initialize ethernet
//
BOOL   LPC32XX_Eth_Deinit(VOID)
{
    OALMSGS((OAL_ETHER && OAL_FUNC), (L"+LPC32XX_Eth_Deinit\r\n"));

	return HWDeInit();
}

//------------------------------------------------------------------------------
//
// LPC32XX_Eth_SendFrame
//
// Send an ethernet frame
//
UINT16 LPC32XX_Eth_SendFrame(UINT8 *pbData, UINT32 length)
{
	UINT32 idx, cidx, fb;

	// Determine number of free buffers and wait for a buffer if needed
	fb = 0;
	while (fb == 0)
	{
		idx = pEregs->txproduceindex;
		cidx = pEregs->txconsumeindex;

		if (idx == cidx)
		{
			// Producer and consumer are the same, all buffers are free
			fb = ENET_MAX_TX_PACKETS;
		}
		else if (cidx > idx)
		{
			fb = (ENET_MAX_TX_PACKETS - 1) -
				((idx + ENET_MAX_TX_PACKETS) - cidx);
		}
		else
		{
			fb = (ENET_MAX_TX_PACKETS - 1) - (cidx - idx);
		}
	}

	// Update descriptor with new frame size
	pTXDesc[idx].control = ((length - 1) | 0x40000000);

	// Move data to buffer
	memcpy((void *) pTXVBuffs [idx], pbData, length);

	// Get next index for transmit data DMA buffer and descriptor
	idx++;
	if (idx >= ENET_MAX_TX_PACKETS)
	{
		idx = 0;
	}
	pEregs->txproduceindex = idx;

	return 0;
}

//------------------------------------------------------------------------------
//
// LPC32XX_Eth_GetFrame
//
// Get ethernet frame
//
UINT16 LPC32XX_Eth_GetFrame(UINT8 *pbData, UINT16 *pLength)
{
	UINT32 idx, length;

	// Determine if a frame has been received
	length = 0;
	idx = pEregs->rxconsumeindex;
	if (pEregs->rxproduceindex != idx)
	{
		// Clear interrupt
		pEregs->intclear = MACINT_RXDONEINTEN;

		// Frame received, get size of RX packet
		length = (pRXStatus[idx].statusinfo & 0x7FF) - 1;

		// Copy data to passed buffer
        memcpy(pbData, (void *) pRXVBuffs [idx], length);

		// Return DMA buffer
		idx++;
		if (idx >= ENET_MAX_TX_PACKETS)
		{
			idx = 0;
		}
		pEregs->rxconsumeindex = (UNS_32) idx;
	}

    // Return size
    *pLength = (UINT16) length;

	return (UINT16) length;
}

//------------------------------------------------------------------------------
//
// LPC32XX_Eth_EnableInts
//
// Enable ethernet interrupts
//
VOID   LPC32XX_Eth_EnableInts()
{
	// Enable EMAC interrupts
    OALMSGS((OAL_ETHER && OAL_FUNC), (L"+LPC32XX_Eth_EnableInts\r\n"));
	pEregs->intclear = 0xFFFF;
	pEregs->intenable = MACINT_RXDONEINTEN;
}

//------------------------------------------------------------------------------
//
// LPC32XX_Eth_DisableInts
//
// Disable ethernet interrupts
//
VOID   LPC32XX_Eth_DisableInts()
{
	// Disable EMAC interrupts
    OALMSGS((OAL_ETHER && OAL_FUNC), (L"+LPC32XX_Eth_DisableInts\r\n"));
	pEregs->intenable = 0;
	pEregs->intclear = 0xFFFF;
}

//------------------------------------------------------------------------------
//
// LPC32XX_Eth_PowerOff
//
// Power down ethernet interface
//
VOID   LPC32XX_Eth_PowerOff()
{
    OALMSGS((OAL_ETHER && OAL_FUNC), (L"+LPC32XX_Eth_PowerOff)\r\n"));
	HWDeInit();
}

//------------------------------------------------------------------------------
//
// LPC32XX_Eth_PowerOn
//
// Power up ethernet interface
//
VOID   LPC32XX_Eth_PowerOn()
{
    OALMSGS((OAL_ETHER && OAL_FUNC), (L"+LPC32XX_Eth_PowerOn)\r\n"));
	HWInit();
}

//------------------------------------------------------------------------------
//
// LPC32XX_Eth_CurrentPacketFilter
//
// Setup ethernet packet filtering
//
VOID   LPC32XX_Eth_CurrentPacketFilter(UINT32 filter)
{
	UINT32 rxfilt;

    OALMSGS((OAL_ETHER && OAL_FUNC), (L"+LPC32XX_Eth_CurrentPacketFilter)\r\n"));

    // Read current filter
    rxfilt = pEregs->rxfliterctrl;

	// All multicast packets
    if ((filter & PACKET_TYPE_ALL_MULTICAST) != 0) {
		pEregs->hashfilterL = 0xFFFFFFFF;
		pEregs->hashfilterh = 0xFFFFFFFF;
		rxfilt |= RXFLTRW_ACCEPTUMULTICAST;
    }
	else
	{
		rxfilt &= ~RXFLTRW_ACCEPTUMULTICAST;
	}

	// Broadcast packets
    if ((filter & PACKET_TYPE_BROADCAST) != 0) {
		rxfilt |= RXFLTRW_ACCEPTUBROADCAST;
    }
	else
	{
		rxfilt &= ~RXFLTRW_ACCEPTUBROADCAST;
	}

	// Promiscous mode
    if ((filter & PACKET_TYPE_PROMISCUOUS) != 0) {
		rxfilt |= (RXFLTRW_ACCEPTUMULTICAST |
			RXFLTRW_ACCEPTUBROADCAST | RXFLTRW_ACCEPTUNICAST);
    }
	else
	{
		rxfilt &= ~RXFLTRW_ACCEPTUNICAST;
	}

	// Enable broadcast and matching address packets
	pEregs->rxfliterctrl = rxfilt;
}

//------------------------------------------------------------------------------
//
// LPC32XX_Eth_MulticastList
//
// Setup ethernet multi-casst addressing
//
BOOL   LPC32XX_Eth_MulticastList(UINT8 *pAddresses, UINT32 count)
{
	OALMSGS((OAL_ETHER && OAL_FUNC), (L"+LPC32XX_Eth_MulticastList)\r\n"));
	return FALSE;
}
