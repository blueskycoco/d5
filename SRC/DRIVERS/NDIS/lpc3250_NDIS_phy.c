#include <Ndis.h>
#include "LPC3250_NDIS_Adapter.h"

#define MII_WR_TOUT			0x00050000  /* MII Write timeout count           */
#define MII_RD_TOUT			0x00050000  /* MII Read timeout count            */

#define PHY_TOUT			0x00010000	//	PHY default timeout

//------------------------------------------------------------------------------
//	PHY Register Descriptions
//------------------------------------------------------------------------------
#define SMSC8700_DEF_ADR		0x00		//	Default SMSC 8700i PHY device address
#define SMSC8700_ID				0x0007C0C0	//	SMSC8700i PHY Identifier	RGUNS: write the real value


//	STD PHY Register addresses
#define PHY_REG_BMCR			0x00		//	Basic Mode Control Register
#define PHY_REG_BMSR			0x01		//	Basic Mode Status Register
#define PHY_REG_IDR1			0x02        //	Extended PHY Identifier 1
#define PHY_REG_IDR2			0x03        //	Extended PHY Identifier 2
#define PHY_REG_ANAR			0x04        //	Extended Auto-Negotiation Advertisement
#define PHY_REG_ANLPAR			0x05        //	Extended Auto-Neg. Link Partner Abitily (Base Page)
#define PHY_REG_ANER			0x06        //	Extended Auto-Neg. Expansion Register
#define PHY_REG_ANNPTR			0x07        //	Auto-Neg. Next Page TX

//	PHY_REG_BMCR description
#define PHY_BMCR_RESET			(1 << 15)	//	Software Reset of the PHY
#define PHY_BMCR_LOOPBACK		(1 << 14)	//	Enable Loopback
#define PHY_BMCR_100MBPS		(1 << 13)	//	Set to 1 for 100 Mbps, to 0 for 10 Mbps
#define PHY_BMCR_AUTONEG		(1 << 12)	//	Select Auto Negotiation
#define PHY_BMCR_POWERDOWN		(1 << 11)	//	Generate Power Down Mode
#define PHY_BMCR_ISOLATE		(1 << 10)	//	Electrical Isolation of PHY from MII
#define PHY_BMCR_RESTARTAUTONEG	(1 <<  9)	//	Restart Auto Negotiation
#define PHY_BMCR_FULLDUPLEX		(1 <<  8)	//	Full duplex if set, Half duplex otherwise
#define PHY_BMCR_COL_TEST_EN	(1 <<  7)	//	Enable Collision test

//	PHY_REG_BMSR description
#define PHY_BMSR_T4_ABLE		(1 << 15)
#define PHY_BMSR_TX_FULL_DUPLEX	(1 << 14)
#define PHY_BMSR_TX_HALF_DUPLEX	(1 << 13)
#define PHY_BMSR_10MBPS_FULL	(1 << 12)
#define PHY_BMSR_10MBPS_HALF	(1 << 11)
#define PHY_BMSR_AUTONEG_COMP	(1 <<  5)
#define PHY_BMSR_REMOTE_FAULT	(1 <<  4)
#define PHY_BMSR_AUTONEG_ABLE	(1 <<  3)
#define PHY_BMSR_LINK_UP		(1 <<  2)
#define PHY_BMSR_JABBER_COND	(1 <<  1)
#define PHY_BMSR_EXT_CAPABILITY	(1 <<  0)

//	PHY_REG_ANAR description
#define PHY_ANAR_NEXT_PAGE_ABLE		(1 << 15)
#define PHY_ANAR_REMOTE_FAULT		(1 << 13)
	#define PHY_ANAR_NO_PAUSE		(0)
	#define PHY_ANAR_ASYM_PAUSE		(1 << 10)
	#define	PHY_ANAR_SYM_PAUSE		(2 << 10)
	#define PHY_ANAR_BOTH_PAUSES	(3 << 10)
#define PHY_ANAR_T4_ABLE			(1 <<  9)
#define PHY_ANAR_100TX_FULL			(1 <<  8)
#define PHY_ANAR_100TX_ABLE			(1 <<  7)
#define PHY_ANAR_10_FULL			(1 <<  6)
#define PHY_ANAR_10_ABLE			(1 <<  5)
#define PHY_ANAR_SELECTOR			(1)

//	PHY_REG_ANLPAR description (Auto Negotiation Link Partner Ability)
#define PHY_ANLPAR_NEXT_PAGE_ABLE	(1 << 15)
#define PHY_ANLPAR_ACK_RECEIVED		(1 << 14)
#define PHY_ANLPAR_REMOTE_FAULT		(1 << 13)
#define PHY_ANLPAR_PAUSE_OP			(1 << 10)
#define PHY_ANLPAR_T4_ABLE			(1 <<  9)
#define PHY_ANLPAR_100TX_FULL		(1 <<  8)
#define PHY_ANLPAR_100TX_ABLE		(1 <<  7)
#define PHY_ANLPAR_10_FULL			(1 <<  6)
#define PHY_ANLPAR_10_ABLE			(1 <<  5)
#define PHY_ANLPAR_SELECTOR			(1)

//	PHY_REG_ANER description (Auto Negotiation Expansion)
#define PHY_ANER_PARALLEL_FAULT			(1 <<  4)
#define PHY_ANER_PARTNER_NEXTPAGE_ABLE	(1 <<  3)
#define PHY_ANER_NEXT_PAGE_ABLE			(1 <<  2)
#define PHY_ANER_PAGE_RECEIVED			(1 <<  1)
#define PHY_ANER_PARTNER_AUTONEG_ABLE	(1 <<  0)


//	PHY SMSC 8700i Vendor Specific Register addresses
#define PHY_8700_REVISION				0x10		//	Silicon Revision Register
#define PHY_8700_MODE_CTRL				0x11		//	Mode Control/Status Register
#define PHY_8700_SPECIAL				0x12		//	Special Modes Register
#define PHY_8700_SYM_ERROR_COUNT		0x1A		//	Symbol Error Counter Register
#define PHY_8700_CTRL_STAT_IND			0x1B		//	Control / Status Indication Register
#define PHY_8700_SPECIAL_INTERN_TEST	0x1C		//	Special Internal Testability Controls
#define PHY_8700_INT_SRC				0x1D		//	Interrupt Source Register
#define PHY_8700_INT_MSK				0x1E		//	Interrupt Mask Register
#define PHY_8700_SPECIAL_CTRL_STAT		0x1F		//	PHY Special Control / Status Register

#define PHY_8700_SPECIAL_RMII			(1 << 14)
#define PHY_8700_SPECIAL_10HD			(0 << 5)
#define PHY_8700_SPECIAL_10FD			(1 << 5)
#define PHY_8700_SPECIAL_100HD			(2 << 5)
#define PHY_8700_SPECIAL_100FD			(3 << 5)
#define PHY_8700_SPECIAL_100HD_AUTONEG	(4 << 5)
#define PHY_8700_SPECIAL_100FD_AUTONEG	(5 << 5)
#define PHY_8700_SPECIAL_POWER_DOWN		(6 << 5)
#define PHY_8700_SPECIAL_AUTONEG		(7 << 5)
#define PHY_8700_SPECIAL_OP_MODE		(0x7 << 5)
#define PHY_8700_SPECIAL_ADDRESS		(0x1F << 0)

/*	write_PHY
	
	Brief:	This board specific function sends a value to the PHY

	Parameters:	PhyReg	Register index
				Value	Value to write to the specified register
*/
void write_PHY (P_ETHERNET_REGS_T pEthernet, BYTE bPhyAddr, DWORD PhyReg, DWORD Value)
{
	unsigned int tout;

	pEthernet->madr = (bPhyAddr << 8) | (PhyReg & 0x0FF);
	pEthernet->mwtd = Value;

	//	Wait until operation completed
	for (tout = 0; tout < MII_WR_TOUT; tout++)
	{
		if ((pEthernet->mind & MIND_BUSY) == 0)
		{
			break;
		}
	}
	if(tout == MII_WR_TOUT)
	{
		RETAILMSG(1, (L"write_PHY Timed out\r\n"));
	}
}

DWORD read_PHY (P_ETHERNET_REGS_T pEthernet, BYTE bPhyAddr, DWORD PhyReg) 
{
	unsigned int tout;

	pEthernet->madr = (bPhyAddr << 8) | (PhyReg & 0x0FF);
	pEthernet->mcmd = MCMD_READ;

	//	Wait until operation completed
	for (tout = 0; tout < MII_RD_TOUT; tout++)
	{
		if ((pEthernet->mind & MIND_BUSY) == 0)
		{
			break;
		}
	}
	if(tout == MII_RD_TOUT)
	{
		RETAILMSG(1, (L"read_PHY Timed out\r\n"));
	}
	pEthernet->mcmd = 0;
	return (pEthernet->mrdd);
}

/*	PHY_CheckLinkOK
	
	Brief:	This function check for the Link Status (Up or Down)

	Return:	0 if the link is down
			1 if the link is up
*/
BOOL PHY_CheckLinkOK(P_ETHERNET_REGS_T pEthernet)
{
	LPC_REG regv = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMSR);
	regv = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMSR);
	return ((regv & PHY_BMSR_LINK_UP) == PHY_BMSR_LINK_UP);
}

BOOL PHY_FullDuplex(P_ETHERNET_REGS_T pEthernet)
{
	LPC_REG regv = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMSR);
	regv = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMSR);
	return	((regv & PHY_BMSR_TX_FULL_DUPLEX) == PHY_BMSR_TX_FULL_DUPLEX) ||
			((regv & PHY_BMSR_10MBPS_FULL) == PHY_BMSR_10MBPS_FULL);
}

BOOL PHY_Speed(P_ETHERNET_REGS_T pEthernet)
{
	LPC_REG regv = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMSR);
	regv = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMSR);
	return	((regv & PHY_BMSR_TX_FULL_DUPLEX) == PHY_BMSR_TX_FULL_DUPLEX) ||
			((regv & PHY_BMSR_TX_HALF_DUPLEX) == PHY_BMSR_TX_HALF_DUPLEX);
}

void PHY_SetDuplex(P_ETHERNET_REGS_T pEthernet,BOOL bFullDuplex)
{
	LPC_REG regv = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMCR);
	regv = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMCR);

	if(bFullDuplex)
	{
		regv |=  PHY_BMCR_FULLDUPLEX;
	}
	else
	{
		regv &= ~PHY_BMCR_FULLDUPLEX;
	}

	write_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMCR, regv);
}

void PHY_SetSpeed(P_ETHERNET_REGS_T pEthernet,BOOL b100Mbps)
{
	LPC_REG regv = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMCR);
	regv = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMCR);

	if(b100Mbps)
	{
		regv |=  PHY_BMCR_100MBPS;
	}
	else
	{
		regv &= ~PHY_BMCR_100MBPS;
	}

	write_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMCR, regv);
}

/*	PHY_SWReset

	Brief:	This function reset the PHY using the software
*/
BOOL PHY_SWReset(P_ETHERNET_REGS_T pEthernet)
{
	int i;

	DEBUGMSG(ZONE_INFO, (L"PHY_SWReset: Put the PHY in reset mode\r\n"));

	//	Put the PHY in reset mode
	write_PHY(pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMCR, PHY_BMCR_RESET);

	//	Wait for hardware reset to end.
	for(i = 0; i < PHY_TOUT; i++)
	{
		if (!(read_PHY(pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMCR) & PHY_BMCR_RESET))
		{
			//	Reset complete
			break;
		}
	}

	if(i == PHY_TOUT)
	{
		RETAILMSG(1, (L"PHY SW Reset failed.\r\n"));
		return FALSE;
	}
	return TRUE;
}

void PHY_HWReset(P_ETHERNET_REGS_T pEthernet)
{
}

/*	PHY_GetID

	Brief:	This function return the PHY ID
*/
DWORD PHY_GetID(P_ETHERNET_REGS_T pEthernet)
{
	DWORD id1,id2;

	id1 = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_IDR1);
	id2 = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_IDR2);

	return ((id1 << 16) | (id2 & 0xFFF0));
}

void PHY_SetInterface(P_ETHERNET_REGS_T pEthernet, BOOL bRMII)
{
	DWORD dwSpecialMode = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_8700_SPECIAL);

	if((dwSpecialMode & PHY_8700_SPECIAL_RMII) != 0)
	{
		DEBUGMSG(ZONE_INFO, (L"RMII Mode Selected in the PHY. "));
		if(bRMII)
		{
			DEBUGMSG(ZONE_INFO, (L"No need to change it.\r\n"));
		}
		else
		{
			DEBUGMSG(ZONE_INFO, (L"Changing to MII: "));
			write_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_8700_SPECIAL, dwSpecialMode & (~PHY_8700_SPECIAL_RMII) );
			DEBUGMSG(ZONE_INFO, (L"DONE\r\n"));
		}
	}
	else
	{
		DEBUGMSG(ZONE_INFO, (L"MII Mode Selected in the PHY. "));
		if(bRMII)
		{
			DEBUGMSG(ZONE_INFO, (L"Changing to RMII: "));
			write_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_8700_SPECIAL, dwSpecialMode | PHY_8700_SPECIAL_RMII);
			DEBUGMSG(ZONE_INFO, (L"DONE\r\n"));
		}
		else
		{
			DEBUGMSG(ZONE_INFO, (L"No need to change it.\r\n"));
		}
	}
}

BOOL PHY_CheckCompatibility(P_ETHERNET_REGS_T pEthernet)
{
	DWORD dwPHYID;

	dwPHYID = PHY_GetID(pEthernet);

	DEBUGMSG(ZONE_INFO, (L"PHY ID: 0x%x", dwPHYID));

	if(dwPHYID != SMSC8700_ID)
	{
		RETAILMSG(1, (L"PHY_CheckCompatibility: this PHY is not supported.\r\n"));
		return FALSE;
	}
	DEBUGMSG(ZONE_INFO, (L"PHY_CheckCompatibility: SMSC 8700i detected.\r\n"));

	return TRUE;
}

/*	PHY_InitLink
*/
BOOL PHY_InitLink(P_ETHERNET_REGS_T pEthernet)
{
	int i;
	DWORD dwRegValue;

	//	Configure the PHY device
	//	Use autonegotiation about the link speed.
	write_PHY(pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMCR, PHY_BMCR_AUTONEG | PHY_BMCR_RESTARTAUTONEG);			//	Set to AutoNeg

	//	Wait to complete Auto_Negotiation.
	for (i = 0; i < PHY_TOUT; i++)
	{
		dwRegValue = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMSR);//	Link status is latched, so read twice to get current value
		dwRegValue = read_PHY (pEthernet, SMSC8700_DEF_ADR, PHY_REG_BMSR);
		if((dwRegValue & PHY_BMSR_AUTONEG_COMP) == PHY_BMSR_AUTONEG_COMP)
		{
			//	Autonegotiation Complete.
			break;
		}
	}
	if(i == PHY_TOUT)
	{
		RETAILMSG(1, (L"AutoNeg Timed out\r\n"));
		return FALSE;
	}
	DEBUGMSG(ZONE_INFO, (L"AutoNeg Set\r\n"));

	return TRUE;
}