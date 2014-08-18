
#include <Ndis.h>
#include "LPC3250_NDIS_Adapter.h"
#include "LPC3250_NDIS_Proto.h"

USHORT	MakeHash(PUCHAR puAddress);
void	SetHash(MINIPORT_ADAPTER *,USHORT);

/*
 Function Name : 	LPC3250_NDIS_MiniportSetInformation
 Description   :
					Required function, by the NDIS. Called by the wrapper to 
					request changes in the state information for the hardware.

  Parameters   :    

					NDIS_HANDLE AdapterContext          handle to context area 
					   NDIS_OID Oid                     OID reference for the set operation
					      PVOID InformationBuffer       buffer containing the ODI specific data
					      ULONG InformationBufferLength length of the information buffer
					     PULONG BytesRead               function sets to the number of bytes 
														it read from the buffer at InformationBuffer
					     PULONG BytesNeeded             function sets this to number of additional bytes 
														it needs to satisfy the request, in case buffer 
														length is not sufficient
  Return Value :
 				NDIS_STATUS     Status      
 */


NDIS_STATUS LPC3xxx_NDIS_MiniportSetInformation(NDIS_HANDLE	MiniportAdapterContext,
												NDIS_OID	Oid,
												PVOID		InformationBuffer,
												ULONG		InformationBufferLength,
												PULONG		BytesRead,
												PULONG		BytesNeeded)
{
	PMINIPORT_ADAPTER    Adapter = (PMINIPORT_ADAPTER) MiniportAdapterContext;
	NDIS_STATUS          Status  = NDIS_STATUS_SUCCESS;
	NDIS_OID            *NdisOid = (NDIS_OID *) InformationBuffer;
	DEBUGMSG(ZONE_INFO, (TEXT("LPC3250 NDIS:==> MiniPort Set Information\r\n")));

	switch(Oid)
	{
        //Current packet filter enabled
        case OID_GEN_CURRENT_PACKET_FILTER:
			Status = MiniPortChangeFilter(Adapter, *NdisOid);
			break;

        //Current LookAhead size.
        case OID_GEN_CURRENT_LOOKAHEAD:
			if(*NdisOid > MAX_FRAME_SIZE)
			{
                Status = NDIS_STATUS_FAILURE;
			}
			else
			{
				DEBUGMSG(ZONE_INFO, (L"CHANGING dwLookAheadBufferSize from %d to %d\r\n",
								Adapter->dwLookAheadBufferSize,
								*NdisOid));
				//Adapter->dwLookAheadBufferSize = *NdisOid;
				*BytesRead         = sizeof(NDIS_OID);
			}
            break;

        //Reset the list of all multicast addresses enabled.
        case OID_802_3_MULTICAST_LIST:
			if((InformationBufferLength % ETH_LENGTH_OF_ADDRESS) != 0)
			{
				Status = NDIS_STATUS_INVALID_LENGTH;
				// We don't know how big it is, assume one entry.
				*BytesNeeded = 	InformationBufferLength + 
								(ETH_LENGTH_OF_ADDRESS - 
								(InformationBufferLength % ETH_LENGTH_OF_ADDRESS));
				*BytesRead = 0;
				break;
			}

			Status = MiniPortChangeAddresses(	Adapter,
												(UCHAR *) NdisOid,
												(UINT) (InformationBufferLength / MAC_ADDRESS_SIZE));
			NdisMoveMemory(	Adapter->tMulticastTable.MulticastTableEntry,
							InformationBuffer,
							InformationBufferLength);
			Adapter->tMulticastTable.MulticastTableEntryCount = InformationBufferLength / ETH_LENGTH_OF_ADDRESS;
			*BytesRead = InformationBufferLength;
			break;

        //Unknown or unsupported Object Identifier.
		default:
			DEBUGMSG(ZONE_INFO, (TEXT("Unknown or Unsupported OID Set Request! OID=%d\r\n"),Oid));

			Status = NDIS_STATUS_NOT_SUPPORTED;
			break;
	}

	DEBUGMSG(ZONE_INFO, (TEXT("LPC3250 NDIS:<== MiniPort Set Information\r\n")));
	return(Status);
}


/*
 Function Name : 	MiniPortChangeFilter
 Description   :	Called by the filter library when a significant change has occured to
					the filter library. This function updates the driver and the hardware 
					filters.
 Parameters    :			MINIPORT_ADAPTER   *Adapter		//Pointer to the adapter structure
                            NDIS_OID            NewFilter	//Filter OID
	
 Return Value  :
					 NDIS_STATUS     Status      
					
*/

NDIS_STATUS MiniPortChangeFilter(PMINIPORT_ADAPTER	pAdapter,
                                 NDIS_OID			NewFilter)
{
	DWORD	dwCmd = pAdapter->pEMACRegs->command;
	DWORD	dwRxFilter = pAdapter->pEMACRegs->rxfliterctrl;

	DEBUGMSG(ZONE_INIT, (TEXT("LPC3250 NDIS==> MiniPort Change Filter\r\n")));
    
    //Check for invalid type.
    if(NewFilter & ~((
		NDIS_PACKET_TYPE_BROADCAST    | 
		NDIS_PACKET_TYPE_MULTICAST    |
		NDIS_PACKET_TYPE_DIRECTED     |
		NDIS_PACKET_TYPE_PROMISCUOUS  |
		NDIS_PACKET_TYPE_ALL_MULTICAST
		)))
        return(NDIS_STATUS_NOT_SUPPORTED);

	//Rcv all directed packets
	if(NewFilter & NDIS_PACKET_TYPE_DIRECTED)
	{
		pAdapter->bPerfectMatch = TRUE;
		dwRxFilter |= RXFILTERCTRL_ACPT_PERFECT_EN;
		DEBUGMSG(ZONE_INFO, (L"MiniPortChangeFilter: Enabling Perfect match\r\n"));
	}
	else
	{
		pAdapter->bPerfectMatch = FALSE;
		RETAILMSG(1, (L"MiniPortChangeFilter: WARNING: Disabling Perfect match packets !!!!!\r\n"));
		dwRxFilter &= ~RXFILTERCTRL_ACPT_PERFECT_EN;
	}

	//Rcv only the multicast with addresses in the list, by clearing the ALMUL in the RCR
	if(NewFilter & NDIS_PACKET_TYPE_MULTICAST)
	{
		pAdapter->bRxMulticastHash = TRUE;
		dwRxFilter |= RXFILTERCTRL_ACPT_MULTI_HASH_EN;
		DEBUGMSG(ZONE_INFO, (L"MiniPortChangeFilter: Enabling Hash Multicast\r\n"));
	}
	else
	{
		pAdapter->bRxMulticastHash = FALSE;
		dwRxFilter &= ~RXFILTERCTRL_ACPT_MULTI_HASH_EN;
		DEBUGMSG(ZONE_INFO, (L"MiniPortChangeFilter: Disabling Hash Multicast\r\n"));
	}

	if(NewFilter & NDIS_PACKET_TYPE_BROADCAST)
	{
		pAdapter->bRxBroadcast = TRUE;   
		dwRxFilter |= RXFILTERCTRL_ACPT_BROADCAST_EN;
		DEBUGMSG(ZONE_INFO, (L"MiniPortChangeFilter: Enabling Broadcast\r\n"));
	}
	else
	{
		pAdapter->bRxBroadcast = FALSE;
		dwRxFilter &= ~RXFILTERCTRL_ACPT_BROADCAST_EN;
		DEBUGMSG(ZONE_INFO, (L"MiniPortChangeFilter: Disabling Broadcast\r\n"));
	}
	
	if(NewFilter & NDIS_PACKET_TYPE_ALL_MULTICAST)
    {
        pAdapter->bRxMulticastAll = TRUE;
		dwRxFilter |= RXFILTERCTRL_ACPT_MULTICAST_EN;
		DEBUGMSG(ZONE_INFO, (L"MiniPortChangeFilter: Enabling All Multicast\r\n"));
    }
    else
	{
		pAdapter->bRxMulticastAll = FALSE;
		dwRxFilter &= ~RXFILTERCTRL_ACPT_MULTICAST_EN;
		DEBUGMSG(ZONE_INFO, (L"MiniPortChangeFilter: Disabling All Multicast\r\n"));
	}
	
    if(NewFilter & NDIS_PACKET_TYPE_PROMISCUOUS)
    {
        pAdapter->bPromiscuous = TRUE;
		dwCmd |= COMMAND_PASSRXFILTER;
		DEBUGMSG(ZONE_INFO, (L"MiniPortChangeFilter: Enabling Promiscuous\r\n"));
		//	RGUNS: Possibly, we might need to enables the 3 bits in the RxFilter, but datasheet states that the PassRxFilter bit bypasses the RxFilter
    }
    else
	{
		pAdapter->bPromiscuous = FALSE;
		dwCmd &= ~COMMAND_PASSRXFILTER;
		DEBUGMSG(ZONE_INFO, (L"MiniPortChangeFilter: Disabling Promiscuous\r\n"));
	}

	pAdapter->pEMACRegs->command = dwCmd;
	pAdapter->pEMACRegs->rxfliterctrl = dwRxFilter;
	
	DEBUGMSG(ZONE_INIT, (TEXT("LPC3250 NDIS<== MiniPort Change Filter\r\n")));
    return(NDIS_STATUS_SUCCESS);
}


NDIS_STATUS MiniPortChangeAddresses(PMINIPORT_ADAPTER	pAdapter,
                                    PUCHAR				AddressList,
                                    UINT				AddressCount)
{
    USHORT       HashBits;

	DEBUGMSG(ZONE_INIT, (TEXT("LPC3250 NDIS==> MiniPort Change Address\r\n")));

	//Clear the current list.
	pAdapter->pEMACRegs->hashfilterL = 0;
	pAdapter->pEMACRegs->hashfilterh = 0;

	//Generate Hash codes for new addresses
	while(AddressCount--)
	{
		HashBits = MakeHash(AddressList);
		SetHash(pAdapter, HashBits);
		AddressList += MAC_ADDRESS_SIZE;
	};

	DEBUGMSG(ZONE_INIT, (TEXT("LPC3250 NDIS<== MiniPort Change Address\r\n")));

	return(NDIS_STATUS_SUCCESS);
}

/*
	This function computes the hash based on the CRC of the MAC Address.
*/
USHORT MakeHash(PUCHAR puAddress)
{
	UINT	crc = (UINT) 0xffffffff;
	USHORT	i;
	USHORT	j;
	USHORT	carry;
	UCHAR	Temp;

	for (i = 0; i < 6; i++)
	{
		Temp = *puAddress++;
		for (j = 0; j < 8; j++)
		{
			carry = ((crc & 0x80000000) ? 1 : 0) ^ (Temp & 0x01);
			crc <<= 1;
			Temp >>= 1;
			if (carry)
				crc = (crc ^ 0x04c11db6) | carry;
		}
	}

	//High order 6 bits are hash value.
	return((USHORT) (crc >> 26));
}

/*
	This function set the right bit in the HashTable of the EMAC based on the HashValue
*/
void    SetHash(PMINIPORT_ADAPTER	pAdapter,
                USHORT				HashValue)
{
	if((HashValue >> 5) == 0)
	{
		//	if HashValue[5]==0, then we are in the first hash register
		pAdapter->pEMACRegs->hashfilterL |= (1 << HashValue);
	}
	else
	{
		//	if HashValue[5]==1, then we are in the second hash register
		pAdapter->pEMACRegs->hashfilterh |= (1 << (HashValue & 0x1F));
	}
}

NDIS_STATUS CopyInfo(UCHAR *InformationBuffer,
                     UCHAR *Source,
                     UINT   BufferLength,
                     UINT   BytesToMove,
                     UINT  *BytesWritten,
                     UINT  *BytesNeeded)
{
	NDIS_STATUS Status = NDIS_STATUS_SUCCESS;

	if(BufferLength < BytesToMove)
	{
		DEBUGMSG(ZONE_INIT, (TEXT("Query Buffer is too small!\r\n")));		
		*BytesNeeded = BytesToMove;
		BytesToMove  = BufferLength;
		Status       = NDIS_STATUS_INVALID_LENGTH;
	}

	NdisMoveMemory(InformationBuffer,Source,BytesToMove);

	*BytesWritten = BytesToMove;	
	return(Status);
}