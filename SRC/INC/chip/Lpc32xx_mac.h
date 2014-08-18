/***********************************************************************
* $Id:: lpc32xx_mac.h 634 2008-04-18 21:31:56Z wellsk                 $
*
* Project: LPC32XX ethernet MAC controller definitions
*
* Description:
*     This file contains the structure definitions and manifest
*     constants for the LPC32xx chip family component:
*         ethernet MAC
*
***********************************************************************
* Software that is described herein is for illustrative purposes only  
* which provides customers with programming information regarding the  
* products. This software is supplied "AS IS" without any warranties.  
* NXP Semiconductors assumes no responsibility or liability for the 
* use of the software, conveys no license or title under any patent, 
* copyright, or mask work right to the product. NXP Semiconductors 
* reserves the right to make changes in the software without 
* notification. NXP Semiconductors also make no representation or 
* warranty that such application will be suitable for the specified 
* use without further testing or modification. 
**********************************************************************/

#ifndef LPC32XX_MAC_H
#define LPC32XX_MAC_H

#include "lpc_types.h"
#include "lpc32xx_chip.h"

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************************
* Ethernet MAC controller register structures
**********************************************************************/

/* Ethernet MAC controller module register structures */ 
typedef struct {
	/* MAC registers */
	volatile UNS_32 mac1;          /* MAC config1 register */
	volatile UNS_32 mac2;          /* MAC config2 register */
	volatile UNS_32 ipgt;          /* MAC b2b interpack gap register */
	volatile UNS_32 ipgr;          /* MAC nb2b interpack gap register */
	volatile UNS_32 clrt;          /* MAC collision retry register */
	volatile UNS_32 maxf;          /* MAC Maximum frame register */
	volatile UNS_32 supp;          /* MAC PHY support register */
	volatile UNS_32 test;          /* MAC test register */
	volatile UNS_32 mcfg;          /* MAC MII mgmt config register */
	volatile UNS_32 mcmd;          /* MAC MII mgmt command register */
	volatile UNS_32 madr;          /* MAC MII mgmt address register */
	volatile UNS_32 mwtd;          /* MAC MII mgmt write register */
	volatile UNS_32 mrdd;          /* MAC MII mgmt read register */
	volatile UNS_32 mind;          /* MAC MII mgmt indicators reg */
	volatile UNS_32 reserved1 [2];
	volatile UNS_32 sa [3];        /* Station address registers */
	volatile UNS_32 reserved2 [45];
	/* Control registers */
	volatile UNS_32 command;
	volatile UNS_32 status;
	volatile UNS_32 rxdescriptor;
	volatile UNS_32 rxstatus;
	volatile UNS_32 rxdescriptornumber;
	volatile UNS_32 rxproduceindex;
	volatile UNS_32 rxconsumeindex;
	volatile UNS_32 txdescriptor;
	volatile UNS_32 txstatus;
	volatile UNS_32 txdescriptornumber;
	volatile UNS_32 txproduceindex;
	volatile UNS_32 txconsumeindex;
	volatile UNS_32 reserved3 [10];
	volatile UNS_32 tsv0;
	volatile UNS_32 tsv1;
	volatile UNS_32 rsv;
	volatile UNS_32 reserved4 [3];
	volatile UNS_32 flowcontrolcounter;
	volatile UNS_32 flowcontrolstatus;
	volatile UNS_32 reserved5 [34];
	/* RX filter registers */
	volatile UNS_32 rxfliterctrl;
	volatile UNS_32 rxfilterwolstatus;
	volatile UNS_32 rxfilterwolclear;
	volatile UNS_32 reserved6;
	volatile UNS_32 hashfilterL;
	volatile UNS_32 hashfilterh;
	volatile UNS_32 reserved7 [882];
	/* Module control registers */
	volatile UNS_32 intstatus;
	volatile UNS_32 intenable;
	volatile UNS_32 intclear;
	volatile UNS_32 Intset;
	volatile UNS_32 reserved8;
	volatile UNS_32 powerdown;
	volatile UNS_32 reserved9;
} ETHERNET_REGS_T, *P_ETHERNET_REGS_T;

/* Structure of a TX/RX descriptor */
typedef struct {
	volatile UNS_32 packet;
	volatile UNS_32 control;
} TXRX_DESC_T;

/* Structure of a RX status entry */
typedef struct {
	volatile UNS_32 statusinfo;
	volatile UNS_32 statushashcrc;
} RX_STATUS_T;

/**********************************************************************
* mac1 register definitions
**********************************************************************/
/* Set this to allow receive frames to be received. Internally the
   MAC synchronize this control bit to the incoming receive stream */
#define MAC1_RECV_ENABLE               _BIT(0)
/* When enabled (set to ??, the MAC will pass all frames regardless
   of type (normal vs. Control). When disabled, the MAC does not pass
   valid Control frames */
#define MAC1_PASS_ALL_RX_FRAMES        _BIT(1)
/* When enabled (set to ??, the MAC acts upon received PAUSE Flow
   Control frames. When disabled, received PAUSE Flow Control frames
   are ignored */
#define MAC1_RX_FLOW_CONTROL           _BIT(2)
/* When enabled (set to ??, PAUSE Flow Control frames are allowed
   to be transmitted. When disabled, Flow Control frames are blocked */
#define MAC1_TX_FLOW_CONTROL           _BIT(3)
/* Setting this bit will cause the MAC Transmit interface to be
   looped back to the MAC Receive interface. Clearing this bit
   results in normal operation */
#define MAC1_LOOPBACK                  _BIT(4)
/* Setting this bit will put the Transmit Function logic in reset */
#define MAC1_RESET_TX                  _BIT(8)
/* Setting this bit resets the MAC Control Sublayer / Transmit logic.
   The MCS logic implements flow control */
#define MAC1_RESET_MCS_TX              _BIT(9)
/* Setting this bit will put the Ethernet receive logic in reset */
#define MAC1_RESET_RX                  _BIT(10)
/* Setting this bit resets the MAC Control Sublayer / Receive logic.
   The MCS logic implements flow control */
#define MAC1_RESET_MCS_RX              _BIT(11)
/* Setting this bit will cause a reset to the random number generator
   within the Transmit Function */
#define MAC1_SIMULATION_RESET          _BIT(14)
/* Setting this bit will put all modules within the MAC in reset
   except the Host Interface */
#define MAC1_SOFT_RESET                _BIT(15)

/**********************************************************************
* mac2 register definitions
**********************************************************************/
/* When enabled (set to ??, the MAC operates in Full-Duplex mode.
   When disabled the MAC operates in Half-Duplex mode */
#define MAC2_FULL_DUPLEX               _BIT(0)
/* When enabled (set to ??, both transmit and receive frame lengths
   are compared to the Length/Type field. If the Length/Type field
   represents a length then the check is performed. Mismatches are
   reported in the StatusInfo word for each received frame */
#define MAC2_FRAME_LENGTH_CHECKING     _BIT(1)
/* When enabled (set to ??, frames of any length are transmitted
   and received */
#define MAC2_HUGH_LENGTH_CHECKING      _BIT(2)
/* This bit determines the number of bytes, if any, of proprietary
   header information that exist on the front of IEEE 802.3 frames.
   When 1, four bytes of header (ignored by the CRC function) are
   added. When 0, there is no proprietary header */
#define MAC2_DELAYED_CRC               _BIT(3)
/* Set this bit to append a CRC to every frame whether padding was
   required or not. Must be set if PAD/CRC ENABLE is set. Clear this
   bit if frames presented to the MAC contain a CRC */
#define MAC2_CRC_ENABLE                _BIT(4)
/* Set this bit to have the MAC pad all short frames. Clear this bit
   if frames presented to the MAC have a valid length. This bit is used
   in conjunction with AUTO PAD ENABLE and VLAN PAD ENABLE */
#define MAC2_PAD_CRC_ENABLE            _BIT(5)
/* Set this bit to cause the MAC to pad all short frames to 64 bytes
   and append a valid CRC. Note: This bit is ignored if
   MAC2_PAD_CRC_ENABLE is cleared */
#define MAC2_VLAN_PAD_ENABLE           _BIT(6)
/* Set this bit to cause the MAC to automatically detect the type of
   frame, either tagged or un-tagged, by comparing the two octets
   following the source address with 0x8100 (VLAN Protocol ID) and
   pad accordingly. Table 14?73 - Pad Operation provides a description
   of the pad function based on the configuration of this register.
   Note: This bit is ignored if PAD / CRC ENABLE is cleared */
#define MAC2_AUTO_DETECT_PAD_ENABLE    _BIT(7)
/* When enabled (set to ??, the MAC will verify the content of the
   preamble to ensure it contains 0x55 and is error-free. A packet
   with an incorrect preamble is discarded. When disabled, no preamble
   checking is performed */
#define MAC2_PURE_PREAMBLE_ENFORCEMENT _BIT(8)
/* When enabled (set to ??, the MAC only allows receive packets
   which contain preamble fields less than 12 bytes in length. When
   disabled, the MAC allows any length preamble as per the Standard */
#define MAC2_LONG_PREAMBLE_ENFORCEMENT _BIT(9)
/* When enabled (set to ??, the MAC will immediately retransmit
   following a collision rather than using the Binary Exponential
   Backoff algorithm as specified in the Standard */
#define MAC2_NO_BACKOFF                _BIT(12)
/* When enabled (set to ??, after the MAC incidentally causes a
   collision during back pressure, it will immediately retransmit
   without backoff, reducing the chance of further collisions and
   ensuring transmit packets get sent */
#define MAC2_BACK_PRESSURE             _BIT(13)
/* When enabled (set to ?? the MAC will defer to carrier indefinitely
   as per the Standard. When disabled, the MAC will abort when the
   excessive deferral limit is reached */
#define MAC2_EXCESS_DEFER              _BIT(14)
   
/**********************************************************************
* ipgt register definitions
**********************************************************************/
/* This is a programmable field representing the nibble time offset
   of the minimum possible period between the end of any transmitted
   packet to the beginning of the next. In Full-Duplex mode, the
   register value should be the desired period in nibble times minus 3.
   In Half-Duplex mode, the register value should be the desired
   period in nibble times minus 6. In Full-Duplex the recommended
   setting is 0x15 (21d), which represents the minimum IPG of 960 ns
   (in 100 Mbps mode) or 9.6 ?s (in 10 Mbps mode). In Half-Duplex the
   recommended setting is 0x12 (18d), which also represents the minimum
   IPG of 960 ns (in 100 Mbps mode) or 9.6 ?s (in 10 Mbps mode) */
#define IPGT_LOAD(n)                   ((n) & 0x7F)

/**********************************************************************
* ipgr register definitions
**********************************************************************/
/* This is a programmable field representing the Non-Back-to-Back
   Inter-Packet-Gap. The recommended value is 0x12 (18d), which
   represents the minimum IPG of 960 ns (in 100 Mbps mode) or 9.6 ?s
   (in 10 Mbps mode) */
#define IPGR_LOAD_PART2(n)             ((n) & 0x7F)
/* This is a programmable field representing the optional carrierSense
   window referenced in IEEE 802.3/4.2.3.2.1 'Carrier Deference'. If
   carrier is detected during the timing of IPGR1, the MAC defers to
   carrier. If, however, carrier becomes active after IPGR1, the MAC
   continues timing IPGR2 and transmits, knowingly causing a collision,
   thus ensuring fair access to medium. Its range of values is 0x0 to
   IPGR2. The recommended value is 0xC (12d) */
#define IPGR_LOAD_PART1(n)             (((n) & 0x7F) << 8)

/**********************************************************************
* clrt register definitions
**********************************************************************/
/* This is a programmable field specifying the number of
   retransmission attempts following a collision before aborting the
   packet due to excessive collisions. The Standard specifies the
   attemptLimit to be 0xF (15d). See IEEE 802.3/4.2.3.2.5. */
#define CLRT_LOAD_RETRY_MAX(n)         ((n) & 0xF)
/* This is a programmable field representing the slot time or
   collision window during which collisions occur in properly
   configured networks. The default value of 0x37 (55d) represents a
   56 byte window following the preamble and SFD. */
#define CLRT_LOAD_COLLISION_WINDOW(n)  (((n) & 0x3F) << 8)

/**********************************************************************
* maxf register definitions
**********************************************************************/
/* This field resets to the value 0x0600, which represents a maximum
   receive frame of 1536 octets. An untagged maximum size Ethernet
   frame is 1518 octets. A tagged frame adds four octets for a total
   of 1522 octets. If a shorter maximum length restriction is desired,
   program this 16 bit field. */
#define MAXF_LOAD_MAX_FRAME_LEN(n)     ((n) & 0xFFFF)

/**********************************************************************
* supp register definitions
**********************************************************************/
/* This bit configures the Reduced MII logic for the current operating
   speed. When set, 100 Mbps mode is selected. When cleared, 10 Mbps
   mode is selected */
#define SUPP_SPEED                     _BIT(8)
/* Reset Reduced MII Logic */
#define SUPP_RESET_RMII                _BIT(11)

/**********************************************************************
* test register definitions
**********************************************************************/
/* This bit reduces the effective PAUSE quanta from 64 byte-times to
   1 byte-time. */
#define TEST_SHORTCUT_PAUSE_QUANTA     _BIT(0)
/* This bit causes the MAC Control sublayer to inhibit transmissions,
   just as if a PAUSE Receive Control frame with a nonzero pause time
   parameter was received. */
#define TEST_PAUSE                     _BIT(1)
/* Setting this bit will cause the MAC to assert backpressure on the
   link. Backpressure causes preamble to be transmitted, raising
   carrier sense. A transmit packet from the system will be sent
   during backpressure. */
#define TEST_BACKPRESSURE              _BIT(2)

/**********************************************************************
* mcfg register definitions
**********************************************************************/
/* Set this bit to cause the MII Management hardware to perform read
   cycles across a range of PHYs. When set, the MII Management
   hardware will perform read cycles from address 1 through the value
   set in PHY ADDRESS[4:0]. Clear this bit to allow continuous reads
   of the same PHY. */
#define MCFG_SCAN_INCREMENT            _BIT(0)
/* Set this bit to cause the MII Management hardware to perform
   read/write cycles without the 32 bit preamble field. Clear this bit
   to cause normal cycles to be performed. Some PHYs support
   suppressed preamble. */
#define MCFG_SUPPRESS_PREAMBLE         _BIT(1)
/* This field is used by the clock divide logic in creating the MII
   Management Clock (MDC) which IEEE 802.3u defines to be no faster
   than 2.5 MHz. Some PHYs support clock rates up to 12.5 MHz,
   however. Refer to Table 14?80 below for the definition of values
   for this field. */
#define MCFG_CLOCK_SELECT(n)           (((n) & 0x7) << 2)
/* MCFG_CLOCK_SELECT macro load values */
#define MCFG_CLOCK_HOST_DIV_4          0
#define MCFG_CLOCK_HOST_DIV_6          2
#define MCFG_CLOCK_HOST_DIV_8          3
#define MCFG_CLOCK_HOST_DIV_10         4
#define MCFG_CLOCK_HOST_DIV_14         5
#define MCFG_CLOCK_HOST_DIV_20         6
#define MCFG_CLOCK_HOST_DIV_28         7
/* This bit resets the MII Management hardware */
#define MCFG_RESET_MII_MGMT            _BIT(15)

/**********************************************************************
* mcmd register definitions
**********************************************************************/
/* This bit causes the MII Management hardware to perform a single
   Read cycle. The Read data is returned in Register MRDD (MII Mgmt
   Read Data). */
#define MCMD_READ                      _BIT(0)
/* This bit causes the MII Management hardware to perform Read cycles
   continuously. This is useful for monitoring Link Fail for example */
#define MCMD_SCAN                      _BIT(1)

/**********************************************************************
* madr register definitions
**********************************************************************/
/* This field represents the 5 bit Register Address field of Mgmt
   cycles. Up to 32 registers can be accessed. */
#define MADR_REGISTER_ADDRESS(n)       ((n) & 0x1F)
/* This field represents the 5 bit PHY Address field of Mgmt
   cycles. Up to 31 PHYs can be addressed (0 is reserved). */
#define MADR_PHY_0ADDRESS(n)           (((n) & 0x1F) << 8)

/**********************************************************************
* mwtd register definitions
**********************************************************************/
/* When written, an MII Mgmt write cycle is performed using the 16 bit
   data and the pre-configured PHY and Register addresses from the
   MII Mgmt Address register (MADR). */
#define MWDT_WRITE(n)                  ((n) & 0xFFFF)

/**********************************************************************
* mrdd register definitions
**********************************************************************/
/* Read mask for MUU read */
#define MRDD_READ_MASK                 0xFFFF

/**********************************************************************
* mind register definitions
**********************************************************************/
/* When ??is returned - indicates MII Mgmt is currently performing
   an MII Mgmt Read or Write cycle. */
#define MIND_BUSY                      _BIT(0)
/* When ??is returned - indicates a scan operation (continuous MII
   Mgmt Read cycles) is in progress. */
#define MIND_SCANNING                  _BIT(1)
/* When ??is returned - indicates MII Mgmt Read cycle has not
   completed and the Read Data is not yet valid. */
#define MIND_NOT_VALID                 _BIT(2)
/* When ??is returned - indicates that an MII Mgmt link fail has
   occurred.*/
#define MIND_MII_LINK_FAIL             _BIT(3)

/**********************************************************************
* command register definitions
**********************************************************************/
/* Enable receive */
#define COMMAND_RXENABLE               _BIT(0)
/* Enable transmit */
#define COMMAND_TXENABLE               _BIT(1)
/* When a ??is written, all datapaths and the host registers are
   reset. The MAC needs to be reset separately. */
#define COMMAND_REG_RESET              _BIT(3)
/* When a ??is written, the transmit datapath is reset. */
#define COMMAND_TXRESET                _BIT(4)
/* When a ??is written, the receive datapath is reset. */
#define COMMAND_RXRESET                _BIT(5)
/* When set to ?? passes runt frames smaller than 64 bytes to
   memory unless they have a CRC error. If ??runt frames are
   filtered out. */
#define COMMAND_PASSRUNTFRAME          _BIT(6)
/* When set to ?? disables receive filtering i.e. all frames
   received are written to memory. */
#define COMMAND_PASSRXFILTER           _BIT(7)
/* Enable IEEE 802.3 / clause 31 flow control sending pause
   frames in full duplex and continuous preamble in half duplex. */
#define COMMAND_TXFLOWCONTROL          _BIT(8)
/* When set to ?? RMII mode is selected; if ?? MII mode is
   selected. */
#define COMMAND_RMII                   _BIT(9)
/* When set to ?? indicates full duplex operation. */
#define COMMAND_FULLDUPLEX             _BIT(10)

/**********************************************************************
* status register definitions
**********************************************************************/
/* If 1, the receive channel is active. If 0, the receive channel is
   inactive. */
#define STATUS_RXACTIVE                _BIT(0)
/* If 1, the transmit channel is active. If 0, the transmit channel is
   inactive. */
#define STATUS_TXACTIVE                _BIT(1)

/**********************************************************************
* tsv0 register definitions
**********************************************************************/
/* The attached CRC in the packet did not match the internally
   generated CRC. */
#define TSV0_CRC_ERROR                 _BIT(0)
/* Indicates the frame length field does not match the actual
   number of data items and is not a type field. */
#define TSV0_LENGTH_CHECK_ERROR        _BIT(1)
/* Indicates that frame type/length field was larger tha 1500 bytes. */
#define TSV0_LENGTH_OUT_OF_RANGE       _BIT(2)
/* Transmission of packet was completed. */
#define TSV0_DONE                      _BIT(3)
/* Packet’s destination was a multicast address. */
#define TSV0_MULTICAST                 _BIT(4)
/* Packet’s destination was a broadcast address. */
#define TSV0_BROADCAST                 _BIT(5)
/* Packet was deferred for at least one attempt, but less than
   an excessive defer. */
#define TSV0_PACKET_DEFER              _BIT(6)
/* Packet was deferred in excess of 6071 nibble times in
   100 Mbps or 24287 bit times in 10 Mbps mode. */
#define TSV0_ESCESSIVE_DEFER           _BIT(7)
/* Packet was aborted due to exceeding of maximum allowed
   number of collisions. */
#define TSV0_ESCESSIVE_COLLISION       _BIT(8)
/* Collision occurred beyond collision window, 512 bit times. */
#define TSV0_LATE_COLLISION            _BIT(9)
/* Byte count in frame was greater than can be represented
   in the transmit byte count field in TSV1. */
#define TSV0_GIANT                     _BIT(10)
/* Host side caused buffer underrun. */
#define TSV0_UNDERRUN                  _BIT(11)
/* Macro: The total number of bytes transferred including
   collided attempts. */
#define TSV0_TOTAL_BYTES(n)            (((n) >> 12) & 0xFFFF)
/* The frame was a control frame. */
#define TSV0_CONTROL_FRAME             _BIT(28)
/* The frame was a control frame with a valid PAUSE opcode. */
#define TSV0_PAUSE                     _BIT(29)
/* Carrier-sense method backpressure was previously applied. */
#define TSV0_BACKPRESSURE              _BIT(30)
/* Frame’s length/type field contained 0x8100 which is the
   VLAN protocol identifier. */
#define TSV0_VLAN                      _BIT(31)

/**********************************************************************
* tsv1 register definitions
**********************************************************************/
/* Macro: The total number of bytes in the frame, not counting the
   collided bytes. */
#define TSV1_TRANSMIT_BYTE_COUNT(n)    ((n) & 0xFFFF)
/* Macro: Number of collisions the current packet incurred during
   transmission attempts. The maximum number of collisions
   (16) cannot be represented. */
#define TSV1_COLLISION_COUNT(n)        (((n) >> 16) & 0xF)

/**********************************************************************
* rsv register definitions
**********************************************************************/
/* Macro: Indicates length of received frame. */
#define RSV_RECEIVED_BYTE_COUNT(n)     ((n) & 0xFFFF)
/* Indicates that a packet was dropped. */
#define RSV_RXDV_EVENT_IGNORED         _BIT(16)
/* Indicates that the last receive event seen was not long
   enough to be a valid packet. */
#define RSV_RXDV_EVENT_PREVIOUSLY_SEEN _BIT(17)
/* Indicates that at some time since the last receive statistics,
   a carrier event was detected. */
#define RSV_CARRIER_EVNT_PREVIOUS_SEEN _BIT(18)
/* Indicates that MII data does not represent a valid receive
   code. */
#define RSV_RECEIVE_CODE_VIOLATION     _BIT(19)
/* The attached CRC in the packet did not match the internally
   generated CRC. */
#define RSV_CRC_ERROR                  _BIT(20)
/* Indicates the frame length field does not match the actual
   number of data items and is not a type field. */
#define RSV_LENGTH_CHECK_ERROR         _BIT(21)
/* Indicates that frame type/length field was larger than 1518 bytes */
#define RSV_LENGTH_OUT_OF_RANGE        _BIT(22)
/* The packet had valid CRC and no symbol errors. */
#define RSV_RECEIVE_OK                 _BIT(23)
/* The packet destination was a multicast address. */
#define RSV_MULTICAST                  _BIT(24)
/* The packet destination was a boardcase address. */
#define RSV_BROADCAST                  _BIT(25)
/* Indicates that after the end of packet another 1-7 bits were
   received. A single nibble, called dribble nibble, is formed
   but not sent out. */
#define RSV_DRIBBLE_NIBBLE             _BIT(26)
/* The frame was a control frame. */
#define RSV_CONTROL_FRAME              _BIT(27)
/* The frame was a control frame with a valid PAUSE opcode. */
#define RSV_PAUSE                      _BIT(28)
/* The current frame was recognized as a Control Frame but
   contains an unknown opcode. */
#define RSV_UNSUPPORTED_OPCODE         _BIT(29)
/* Frame’s length/type field contained 0x8100 which is the
   VLAN protocol identifier. */
#define RSV_VLAN                       _BIT(30)

/**********************************************************************
* flowcontrolcounter register definitions
**********************************************************************/
/* Macro: In full duplex mode the MirrorCounter specifies the number
   of cycles before re-issuing the Pause control frame. */
#define FCCR_MIRRORCOUNTER(n)          ((n) & 0xFFFF)
/* Macro: In full-duplex mode the PauseTimer specifies the value
   that is inserted into the pause timer field of a pause flow
   control frame. In half duplex mode the PauseTimer
   specifies the number of backpressure cycles. */
#define FCCR_PAUSETIMER(n)             (((n) >> 16) & 0xFFFF)

/**********************************************************************
* flowcontrolstatus register definitions
**********************************************************************/
/* Macro: In full duplex mode this register represents the current
   value of the datapath’s mirror counter which counts up to
   the value specified by the MirrorCounter field in the
   FlowControlCounter register. In half duplex mode the
   register counts until it reaches the value of the PauseTimer
   bits in the FlowControlCounter register. */
#define FCCR_MIRRORCOUNTERCURRENT(n)   ((n) & 0xFFFF)

/**********************************************************************
* rxfliterctrl, rxfilterwolstatus, and rxfilterwolclear shared
* register definitions
**********************************************************************/
/* Unicast frame control */
#define RXFLTRW_ACCEPTUNICAST          _BIT(0)
/* Broadcase frame control. */
#define RXFLTRW_ACCEPTUBROADCAST       _BIT(1)
/* Multicast frame control */
#define RXFLTRW_ACCEPTUMULTICAST       _BIT(2)
/* Imperfect unicast frame control */
#define RXFLTRW_ACCEPTUNICASTHASH      _BIT(3)
/* Imperfect multicast frame control */
#define RXFLTRW_ACCEPTUMULTICASTHASH   _BIT(4)
/* Perfect frame control */
#define RXFLTRW_ACCEPTPERFECT          _BIT(5)

/**********************************************************************
* rxfliterctrl register definitions
**********************************************************************/
/* When set to ?? the result of the magic packet filter will
   generate a WoL interrupt when there is a match. */
#define RXFLTRWSTS_MAGICPACKETENWOL    _BIT(12)
/* When set to ?? the result of the perfect address
   matching filter and the imperfect hash filter will
   generate a WoL interrupt when there is a match. */
#define RXFLTRWSTS_RXFILTERENWOL       _BIT(13)

/**********************************************************************
* rxfilterwolstatus register definitions
**********************************************************************/
/* When the value is ?? the receive filter caused WoL. */
#define RXFLTRWSTS_RXFILTERWOL         _BIT(7)
/* When the value is ?? the magic packet filter caused WoL. */
#define RXFLTRWSTS_MAGICPACKETWOL      _BIT(8)

/**********************************************************************
* rxfilterwolclear register definitions
**********************************************************************/
/* When a ??is written to one of these bits (7 and/or 8),
   the corresponding status bit in the rxfilterwolstatus
   register is cleared. */
#define RXFLTRWCLR_RXFILTERWOL         RXFLTRWSTS_RXFILTERWOL
#define RXFLTRWCLR_MAGICPACKETWOL      RXFLTRWSTS_MAGICPACKETWOL

/**********************************************************************
* intstatus, intenable, intclear, and Intset shared register
* definitions
**********************************************************************/
/* Interrupt trigger on receive buffer overrun or descriptor underrun
   situations. */
#define MACINT_RXOVERRUNINTEN          _BIT(0)
/* Enable for interrupt trigger on receive errors. */
#define MACINT_RXERRORONINT            _BIT(1)
/* Enable for interrupt triggered when all receive descriptors have
   been processed i.e. on the transition to the situation where
   ProduceIndex == ConsumeIndex. */
#define MACINT_RXFINISHEDINTEN         _BIT(2)
/* Enable for interrupt triggered when a receive descriptor has
   been processed while the Interrupt bit in the Control field of the
   descriptor was set. */
#define MACINT_RXDONEINTEN             _BIT(3)
/* Enable for interrupt trigger on transmit buffer or descriptor
   underrun situations. */
#define MACINT_TXUNDERRUNINTEN         _BIT(4)
/* Enable for interrupt trigger on transmit errors. */
#define MACINT_TXERRORINTEN            _BIT(5)
/* Enable for interrupt triggered when all transmit descriptors
   have been processed i.e. on the transition to the situation
   where ProduceIndex == ConsumeIndex. */
#define MACINT_TXFINISHEDINTEN         _BIT(6)
/* Enable for interrupt triggered when a descriptor has been
   transmitted while the Interrupt bit in the Control field of the
   descriptor was set. */
#define MACINT_TXDONEINTEN             _BIT(7)
/* Enable for interrupt triggered by the SoftInt bit in the IntStatus
   register, caused by software writing a 1 to the SoftIntSet bit in
   the IntSet register. */
#define MACINT_SOFTINTEN               _BIT(12)
/* Enable for interrupt triggered by a Wakeup event detected by
   the receive filter. */
#define MACINT_WAKEUPINTEN             _BIT(13)

/**********************************************************************
* powerdown register definitions
**********************************************************************/
/* If true, all AHB accesses will return a read/write error,
   except accesses to the PowerDown register. */
#define POWERDOWN_MACAHB               _BIT(31)

/* Macro pointing to ethernet MAC controller registers */
#define ENETMAC ((ETHERNET_REGS_T *)(ETHERNET_BASE))

#ifdef __cplusplus
}
#endif
/*************************************************************************************

Add by JJ ucdragon
**************************************************************************************
*/
//! \brief Ethernet Receive and Transmit Descriptor
typedef struct
{
	LPC_REG	PacketAddress;		//!<	PacketAddress		0x0			Base Address of the data buffer for storing receive/containing transmit data
	LPC_REG	Control;			//!<	Control				0x4			Control information
}LPCS_ETH_DESC, *LPCPS_ETH_DESC;

//! \brief Ethernet Receive Status
typedef struct
{
	LPC_REG	StatusInfo;			//!<	StatusInfo			0x0			Receive status return flags
	LPC_REG	StatusHashCRC;		//!<	StatusHashCRC		0x4			The concatenation of the destination address hash CRC and the source address hash CRC
}LPCS_ETH_RX_STATUS, *LPCPS_ETH_RX_STATUS;

//! \brief Ethernet Transmit Status
typedef struct
{
	LPC_REG StatusInfo;			//!<	StatusInfo			0x0			Transmit status return flags
}LPCS_ETH_TX_STATUS, *LPCPS_ETH_TX_STATUS;

//#pragma pack()

//	MAC Configuration register 1 (MAC1 - address 0x3106 0000)
#define MAC1_SOFT_RESET					(1 << 15)
#define MAC1_SIMULATION_RESET			(1 << 14)
#define MAC1_RESET_MCS_RX				(1 << 11)
#define MAC1_RESET_RX					(1 << 10)
#define MAC1_RESET_MCS_TX				(1 <<  9)
#define MAC1_RESET_TX					(1 <<  8)
#define MAC1_LOOPBACK					(1 <<  4)
#define MAC1_TX_FLOW_CTRL				(1 <<  3)
#define MAC1_RX_FLOW_CTRL				(1 <<  2)
#define MAC1_PASS_ALL_RCV				(1 <<  1)
#define MAC1_RECEIVE_EN					(1 <<  0)

//	MAC Configuration register 2 (MAC2 - address 0x3106 0004)
#define MAC2_EXCESS_DEFER				(1 << 14)
#define MAC2_BACK_PRESSURE_NO_BO		(1 << 13)
#define MAC2_NO_BACKOFF					(1 << 12)
#define MAC2_LONG_PREAMBLE				(1 <<  9)
#define MAC2_PURE_PREAMBLE				(1 <<  8)
#define MAC2_AUTO_DETECT_PAD_EN			(1 <<  7)
#define MAC2_VLAN_PAD_EN				(1 <<  6)
#define MAC2_PAD_CRC_EN					(1 <<  5)
#define MAC2_CRC_EN						(1 <<  4)
#define MAC2_DELAYED_CRC				(1 <<  3)
#define MAC2_HUGE_FRAME_EN				(1 <<  2)
#define MAC2_FRAME_LEN_CHECK			(1 <<  1)
#define MAC2_FULL_DUPLEX				(1 <<  0)

//	Back-to-back Inter-packet-gap register (IPGT - address 0x3106 0008)
#define IPGT_INTERPACKET_GAP_MSK		(0x7F << 0)

#define IPGT_FULL_DUP					0x00000015	//	Recommended value for Full Duplex
#define IPGT_HALF_DUP					0x00000012	//	Recommended value for Half Duplex

//	Non Back-to-back Inter-packet-gap register (IPGR - address 0x3106 000C)
#define IPGR_INTERPACKET_GAP_1_MSK		(0x7F << 8)
#define IPGR_INTERPACKET_GAP_2_MSK		(0x7F << 0)

#define IPGR_DEF						0x00000C12  //	Recommended value

//	Collision Window / Retry register (CLRT - address 0x3106 0010)
#define CLRT_COLLISION_WINDOW			(0x3F << 8)
#define CLRT_RETRANSMISSION_MAXIMUM		(0x0F << 0)

#define CLRT_DEF						0x0000370F  // Default value

//	Maximum Frame register (MAXF - address 0x3106 0014)
#define MAXF_MAXIMUM_FRAME_LEN			(0xFFFF << 0)

//	PHY Support register (SUPP - address 0x3106 0018)
#define SUPP_SPEED_100MBPS				(1 << 8)

//	Test register (TEST - address 0x3106 )
#define TEST_BACKPRESSURE				(1 << 2)
#define TEST_PAUSE						(1 << 1)
#define SHORTCUT_PAUSE_QUANTA			(1 << 0)

//	MII Mgmt Configuration register (MCFG - address 0x3106 0020)
#define MCFG_RESET_MGMT					(1 << 15)
#define MCFG_CLK_SEL_MSK				(7 <<  2)
	#define MCFG_HCLK_DIV_4				(1 <<  2)
	#define MCFG_HCLK_DIV_6				(2 <<  2)
	#define MCFG_HCLK_DIV_8				(3 <<  2)
	#define MCFG_HCLK_DIV_10			(4 <<  2)
	#define MCFG_HCLK_DIV_14			(5 <<  2)
	#define MCFG_HCLK_DIV_20			(6 <<  2)
	#define MCFG_HCLK_DIV_28			(7 <<  2)
#define MCFG_SUPPRESS_PREAMBLE			(1 <<  1)
#define MCFG_SCAN_INCREMENT				(1 <<  0)

//	MII Mgmt Command register (MCMD - address 0x3106 0024)
#define MCMD_SCAN						(1 << 1)
#define MCMD_READ						(1 << 0)

//	MII Mgmt Address register (MADR - address 0x3106 0028)
#define MADR_PHY_ADDRESS_MSK			(0x1F << 8)
#define MADR_REGISTER_ADDRESS_MSK		(0x1F << 0)

//	MII Mgmt Write Data register (MWTD - address 0x3106 002C)
#define MWTD_WRITE_DATA_MSK				(0xFF << 0)

//	MII Mgmt Read Data register (MRDD - address 0x3106 0030)
#define MRDD_READ_DATA_MSK				(0xFF << 0)

//	MII Mgmt Indicators register (MIND - address 0x3106 0034)
#define MIND_LINK_FAIL					(1 << 3)
#define MIND_NOT_VALID					(1 << 2)
#define MIND_SCANNING					(1 << 1)
#define MIND_BUSY						(1 << 0)

//	Station Address register (SA0 - address 0x3106 0040)
#define SA0_1ST_OCTET					(8)
#define SA0_1ST_OCTET_MSK				(0xFF << SA0_1ST_OCTET)
#define SA0_2ND_OCTET					(0)
#define SA0_2ND_OCTET_MSK				(0xFF << SA0_2ND_OCTET)
//	Station Address register (SA1 - address 0x3106 0044)
#define SA1_3RD_OCTET					(8)
#define SA1_3RD_OCTET_MSK				(0xFF << SA1_3RD_OCTET)
#define SA1_4TH_OCTET					(0)
#define SA1_4TH_OCTET_MSK				(0xFF << SA1_4TH_OCTET)
//	Station Address register (SA2 - address 0x3106 0048)
#define SA2_5TH_OCTET					(8)
#define SA2_5TH_OCTET_MSK				(0xFF << SA2_5TH_OCTET)
#define SA2_6TH_OCTET					(0)
#define SA2_6TH_OCTET_MSK				(0xFF << SA2_6TH_OCTET)

//	Command register (Command - address 0x3106 0100)
#define COMMAND_FULLDUPLEX				(1 << 10)
#define COMMAND_RMII					(1 <<  9)
#define COMMAND_TXFLOWCTRL				(1 <<  8)
#define COMMAND_PASSRXFILTER			(1 <<  7)
#define COMMAND_PASSRUNTFRAME			(1 <<  6)
#define COMMAND_RXRESET					(1 <<  5)
#define COMMAND_TXRESET					(1 <<  4)
#define COMMAND_REGRESET				(1 <<  3)
#define COMMAND_TX_EN					(1 <<  1)
#define COMMAND_RX_EN					(1 <<  0)

//	Status register (Status - address 0x3106 0104)
#define STATUS_TX						(1 <<  1)
#define STATUS_RX						(1 <<  0)

//	Receive Descriptor Base Address register (RxDescriptor - address 0x3106 0108)
#define RXDESC_RXDESC					(2)
#define RXDESC_MASK						(0x3F << RXDESC_RXDESC)

//	Receive Status Base Address register (RxStatus - address 0x3106 010C)
#define RXSTATUS_RXSTATUS				(3)
#define RXSTATUS_MASK					(0x1F << RXSTATUS_RXDESC)

//	Receive Number of Descriptors register (RxDescriptorNumber - address 0x3106 0110)
#define RXDESCNUM_MASK					(0xFFFF)

//	Receive Produce Index register (RxProduceIndex - address 0x3106 0114)
#define RXPRODINDEX_MASK				(0xFFFF)

//	Receive Consume Index register (RXConsumeIndex - address 0x3106 0118)
#define RXCONSUMEINDEX_MASK				(0xFFFF)

//	Transmit Descriptor Base Address register (TxDescriptor - address 0x3106 011C)
#define TXDESC_RXDESC					(2)
#define TXDESC_MASK						(0x3F << TXDESC_RXDESC)

//	Transmit Status Base Address register (TxStatus - address 0x3106 0120)
#define TXSTATUS_RXSTATUS				(3)
#define TXSTATUS_MASK					(0x1F << TXSTATUS_RXSTATUS)

//	Transmit Number of Descriptors register (TxDescriptorNumber - address 0x3106 0124)
#define TXDESCNUM_MASK					(0xFFFF)

//	Transmit Produce Index register (TxProduceIndex - address 0x3106 0128)
#define TXPRODINDEX_MASK				(0xFFFF)

//	Transmit Consume Index register (TxConsumeIndex - address 0x3106 012C)
#define TXCONSUMEINDEX_MASK				(0xFFFF)

//	Transmit Status Vector 0 register (TSV0 - address 0x3106 0158)
#define TSV0_VLAN						(1 << 31)
#define TSV0_BACKPRESSURE				(1 << 30)
#define TSV0_PAUSE						(1 << 29)
#define TSV0_CONTROLFRAME				(1 << 28)
#define TSV0_TOTAL_BYTES_OFFSET			(12)
#define TSV0_TOTAL_BYTES_MASK			(0xFFFF << TSV0_TOTAL_BYTES_OFFSET)
#define TSV0_UNDERRUN					(1 << 11)
#define TSV0_GIANT						(1 << 10)
#define TSV0_LATE_COLLISION				(1 <<  9)
#define TSV0_EXCESSIVE_COLLISION		(1 <<  8)
#define TSV0_EXCESSIVE_DEFER			(1 <<  7)
#define TSV0_PACKET_DEFER				(1 <<  6)
#define TSV0_BROADCAST					(1 <<  5)
#define TSV0_MULTICAST					(1 <<  4)
#define TSV0_DONE						(1 <<  3)
#define TSV0_LEN_OUT_OF_RANGE			(1 <<  2)
#define TSV0_LEN_CHECK_ERROR			(1 <<  1)
#define TSV0_CRC_ERROR					(1 <<  0)

//	Transmit Status Vector 1 register (TSV1 - address 0x3106 015C)
#define TSV1_TXCOLCOUNT_OFFSET			(16)
#define TSV1_TXCOLCOUNT_MASK			(0x0F << TSV1_TXCOLCOUNT_OFFSET)
#define TSV1_TXBYTECOUNT_OFFSET			(0)
#define TSV1_TXBYTECOUNT_MASK			(0xFFFF << TSV1_TXBYTECOUNT_OFFSET)

//	Receive Status Vector register (RSV - address 0x3106 0160)
#define RSV_VLAN						(1 << 30)
#define RSV_UNSUPPORTED_OP				(1 << 29)
#define RSV_PAUSE						(1 << 28)
#define RSV_CTRL_FRAME					(1 << 27)
#define RSV_DRIBBLE_NIBBLE				(1 << 26)
#define RSV_BROADCAST					(1 << 25)
#define RSV_MULTICAST					(1 << 24)
#define RSV_RECEIVE_OK					(1 << 23)
#define RSV_LEN_OUT_OF_RANGE			(1 << 22)
#define RSV_LEN_CHECK_ERROR				(1 << 21)
#define RSV_CRC_ERROR					(1 << 20)
#define RSV_RECEIVED_CODE_VIOLATION		(1 << 19)
#define RSV_CARRIER_EVT_PRV_SEEN		(1 << 18)
#define RSV_RXDV_EVT_PRV_SEEN			(1 << 17)
#define RSV_PCKT_PRV_IGNORED			(1 << 16)
#define RSV_RCV_BYTES_CNT_OFFSET		(0)
#define RSV_RCV_BYTES_CNT_MASK			(0xFFFF << RSV_RCV_BYTES_CNT_OFFSET)

//	Flow Control Counter register (FlowControlCounter - address 0x3106 0170)
#define FLWCTRLCOUNT_MIRROR_MASK		(0xFFFF)
#define FLWCTRLCOUNT_PAUSE_TIMER		(0xFFFF0000)

//	Flow Control Status register (FlowControlStatus - address 0x3106 8174)
#define FLWCTRLSTATUS_MIRROR_CURR		(0xFFFF)

//	Receive Filter Control register (RxFilterCtrl - address 0x3106 0200)
#define RXFILTERCTRL_EN_WOL				(1 << 13)
#define RXFILTERCTRL_MAGIC_PCKT_WOL_EN	(1 << 12)
#define RXFILTERCTRL_ACPT_PERFECT_EN	(1 <<  5)
#define RXFILTERCTRL_ACPT_MULTI_HASH_EN	(1 <<  4)
#define RXFILTERCTRL_ACPT_UNIHASH_EN	(1 <<  3)
#define RXFILTERCTRL_ACPT_MULTICAST_EN	(1 <<  2)
#define RXFILTERCTRL_ACPT_BROADCAST_EN	(1 <<  1)
#define RXFILTERCTRL_ACPT_UNICAST_EN	(1 <<  0)

//	Receive Filter WoL Status register (RxFilterWoLStatus - address 0x3106 0204)
//	Receive Filter WoL Clear register (RxFilterWoLClear - address 0x3106 0208)
#define RXFILTERWOL_MAGICPACKET			(1 << 8)
#define RXFILTERWOL_RXFILTER			(1 << 7)
#define RXFILTERWOL_ACCEPTPERFECT		(1 << 5)
#define RXFILTERWOL_ACCEPTMULTIHASH		(1 << 4)
#define RXFILTERWOL_ACCEPTUNIHAS		(1 << 3)
#define RXFILTERWOL_ACCEPTMULTICAST		(1 << 2)
#define RXFILTERWOL_ACCEPTBROADCAST		(1 << 1)
#define RXFILTERWOL_ACCEPTUNICAST		(1 << 0)

//	Interrupt Status register (IntStatus - address 0x3106 0FE0)
//	Interrupt Enable register (intEnable - address 0x3106 0FE4)
//	Interrupt Clear register (IntClear - address 0x3106 0FE8)
//	Interrupt Set register (IntSet - address 0x3106 0FEC)
#define INT_WAKEUP						(1 << 13)
#define INT_SOFT						(1 << 12)
#define INT_TXDONE						(1 <<  7)
#define INT_TXFINISHED					(1 <<  6)
#define INT_TXERROR						(1 <<  5)
#define INT_TXUNDERRUN					(1 <<  4)
#define INT_RXDONE						(1 <<  3)
#define INT_RXFINISHED					(1 <<  2)
#define INT_RXERROR						(1 <<  1)
#define INT_RXOVERRUN					(1 <<  0)

#define INT_TX_INT						(INT_TXDONE | INT_TXFINISHED | INT_TXERROR | INT_TXUNDERRUN)
#define INT_RX_INT						(INT_RXDONE | INT_RXFINISHED | INT_RXERROR | INT_RXOVERRUN)

//	Power Down register (PowerDown - address 0x3106 0FF4)
#define POWERDOWN_MACAHB				(1 << 31)


/////// RX and TX Descriptors ///////

//	Receive Descriptor Control Word
#define ETH_RX_DESC_INTERRUPT			(1 << 31)
#define ETH_RX_DESC_SIZE				(0x7FF << 0)	//	This is the size of the buffer, not the frame

//	Transmit Descriptor Control Word
#define ETH_TX_DESC_INTERRUPT			(1 << 31)
#define ETH_TX_DESC_LAST				(1 << 30)
#define ETH_TX_DESC_CRC					(1 << 29)
#define ETH_TX_DESC_PAD					(1 << 28)
#define ETH_TX_DESC_HUGE				(1 << 27)
#define ETH_TX_DESC_OVERRIDE			(1 << 26)
#define ETH_TX_DESC_SIZE				(0x7FF << 0)	//	This is the size of the buffer to send, aka the frame size

//	Receive Status Information Word
#define ETH_RX_STATUS_INFO_ERROR		(1 << 31)
#define ETH_RX_STATUS_INFO_LASTFLAG		(1 << 30)
#define ETH_RX_STATUS_INFO_NODESC		(1 << 29)
#define ETH_RX_STATUS_INFO_OVERRUN		(1 << 28)
#define ETH_RX_STATUS_INFO_ALIGN_ERR	(1 << 27)
#define ETH_RX_STATUS_INFO_RANGE_ERR	(1 << 26)
#define ETH_RX_STATUS_INFO_LENGTH_ERR	(1 << 25)
#define ETH_RX_STATUS_INFO_SYMBOL_ERR	(1 << 24)
#define ETH_RX_STATUS_INFO_CRC_ERR		(1 << 23)
#define ETH_RX_STATUS_INFO_BROADCAST	(1 << 22)
#define ETH_RX_STATUS_INFO_MULTICAST	(1 << 21)
#define ETH_RX_STATUS_INFO_FAILFILTER	(1 << 20)
#define ETH_RX_STATUS_INFO_VLAN			(1 << 19)
#define ETH_RX_STATUS_INFO_CTRL_FRM		(1 << 18)
#define ETH_RX_STATUS_INFO_RX_SIZE		(0)
#define ETH_RX_STATUS_INFO_RX_SIZE_MSK	(0x7FF << ETH_RX_STATUS_INFO_RX_SIZE)

//	Transmit Status Information Word
#define ETH_TX_STATUS_INFO_ERROR		(1 << 31)
#define ETH_TX_STATUS_INFO_NODESC		(1 << 30)
#define ETH_TX_STATUS_INFO_UNDERRUN		(1 << 29)
#define ETH_TX_STATUS_INFO_LATECOLL		(1 << 28)
#define ETH_TX_STATUS_INFO_EXCESSCOLL	(1 << 27)
#define ETH_TX_STATUS_INFO_EXCESSDIFFER	(1 << 26)
#define ETH_TX_STATUS_INFO_DEFER		(1 << 25)
#define ETH_TX_STATUS_INFO_COLL_CNT		(21)
#define ETH_TX_STATUS_INFO_COLL_CNT_MSK	(0x0F << ETH_TX_STATUS_INFO_COLL_CNT)

//	Receive Status HashCRC Word
#define ETH_RX_STATUSHASH_SRCADD_OFFSET	(0)
#define ETH_RX_STATUSHASH_SRCADD_MASK	(0x1FF << ETH_RX_STATUSHASH_SRCADD_OFFSET)
#define ETH_RX_STATUSHASH_DSTADD_OFFSET	(16)
#define ETH_RX_STATUSHASH_DSTADD_MASK	(0x1FF << ETH_RX_STATUSHASH_DSTADD_MASK)

#endif /* LPC32XX_MAC_H */ 
