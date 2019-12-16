/**
 * @file
 * Ethernet Interface for standalone applications (without RTOS) - works only for 
 * ethernet polling mode (polling for ethernet frame reception)
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#include "lwip/mem.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "stm32f2x7_eth.h"
#include "bsp_eth.h"
#include <string.h>

/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'


/* Ethernet Rx & Tx DMA Descriptors */
extern ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB], DMATxDscrTab[ETH_TXBUFNB];

/* Ethernet Driver Receive buffers  */
extern uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE]; 

/* Ethernet Driver Transmit buffers */
extern uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE]; 

/* Global pointers to track current transmit and receive descriptors */
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

/* Global pointer for last received frame infos */
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;




/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
#ifdef CHECKSUM_BY_HARDWARE
  int i; 
#endif
  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] =  MAC_ADDR0;
  netif->hwaddr[1] =  MAC_ADDR1;
  netif->hwaddr[2] =  MAC_ADDR2;
  netif->hwaddr[3] =  MAC_ADDR3;
  netif->hwaddr[4] =  MAC_ADDR4;
  netif->hwaddr[5] =  MAC_ADDR5;
  
  /* initialize MAC address in ethernet MAC */ 
  ETH_MACAddressConfig(ETH_MAC_Address0, netif->hwaddr); 

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

  /* Initialize Tx Descriptors list: Chain Mode */
  ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
  /* Initialize Rx Descriptors list: Chain Mode  */
  ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
  
#ifdef CHECKSUM_BY_HARDWARE
  /* Enable the TCP, UDP and ICMP checksum insertion for the Tx frames */
  for(i=0; i<ETH_TXBUFNB; i++)
    {
      ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
    }
#endif

   /* Note: TCP, UDP, ICMP checksum checking for received frame are enabled in DMA config */

  /* Enable MAC and DMA transmission and reception */
  ETH_Start();

}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  struct pbuf *q;
  int framelength = 0;
  u8 *buffer =  (u8 *)(DMATxDescToSet->Buffer1Addr);
  
  /* copy frame from pbufs to driver buffers */
  for(q = p; q != NULL; q = q->next) 
  {
    memcpy((u8_t*)&buffer[framelength], q->payload, q->len);
	framelength = framelength + q->len;
  }
  
  /* Note: padding and CRC for transmitted frame 
     are automatically inserted by DMA */

  /* Prepare transmit descriptors to give to DMA*/ 
  ETH_Prepare_Transmit_Descriptors(framelength);

  return ERR_OK;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf * low_level_input(struct netif *netif)
{
  struct pbuf *p, *q;
  u16_t len;
  int l =0;
  FrameTypeDef frame;
  u8 *buffer;
  uint32_t i=0;
  __IO ETH_DMADESCTypeDef *DMARxNextDesc;
  
  
  p = NULL;
  
  /* get received frame */
  frame = ETH_Get_Received_Frame();
  
  /* Obtain the size of the packet and put it into the "len" variable. */
  len = frame.length;
  buffer = (u8 *)frame.buffer;
  
  /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
  p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
  
  /* copy received frame to pbuf chain */
  if (p != NULL)
  {
    for (q = p; q != NULL; q = q->next)
    {
      memcpy((u8_t*)q->payload, (u8_t*)&buffer[l], q->len);
      l = l + q->len;
    }    
  }
  
  /* Release descriptors to DMA */
  /* Check if frame with multiple DMA buffer segments */
  if (DMA_RX_FRAME_infos->Seg_Count > 1)
  {
    DMARxNextDesc = DMA_RX_FRAME_infos->FS_Rx_Desc;
  }
  else
  {
    DMARxNextDesc = frame.descriptor;
  }
  
  /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
  for (i=0; i<DMA_RX_FRAME_infos->Seg_Count; i++)
  {  
    DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
    DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
  }
  
  /* Clear Segment_Count */
  DMA_RX_FRAME_infos->Seg_Count =0;
  
  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)  
  {
    /* Clear RBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_RBUS;
    /* Resume DMA reception */
    ETH->DMARPDR = 0;
  }
  return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
err_t ethernetif_input(struct netif *netif)
{
  err_t err;
  struct pbuf *p;

  /* move received packet into a new pbuf */
  p = low_level_input(netif);

  /* no packet could be read, silently ignore this */
  if (p == NULL) return ERR_MEM;

  /* entry point to the LwIP stack */
  err = netif->input(p, netif);
  
  if (err != ERR_OK)
  {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
    pbuf_free(p);
    p = NULL;
  }
  return err;
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));
  
#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/**
  * @brief  This function handles Ethernet link status.
  * @param  None
  * @retval None
  */
extern uint8_t EthInitStatus;
extern void ETH_Reinit(void);
extern ETH_InitTypeDef ETH_InitStructure;
void Eth_Link_ITHandler(struct netif *netif)
{
  /* Check whether the link interrupt has occurred or not */
  if(((ETH_ReadPHYRegister(DP83848_PHY_ADDRESS, PHY_MISR)) & PHY_LINK_STATUS) != 0){/*������ж�*/
		
    uint16_t status  = ETH_ReadPHYRegister(DP83848_PHY_ADDRESS, PHY_BSR);  
    if(status & (PHY_AutoNego_Complete | PHY_Linked_Status)){/*��⵽��������*/
			
			if(EthInitStatus == 0){/*֮ǰδ�ɹ���ʼ����*/
				
				/*Reinit PHY*/
				ETH_Reinit();
			}
			else{/*֮ǰ�Ѿ��ɹ���ʼ��*/
				/*set link up for re link callbalk function*/
					
				netif_set_link_up(netif);	
			}
		}
		else{/*���߶Ͽ�*/
			
			/*set link down for re link callbalk function*/
			netif_set_link_down(netif);
		}
  }
}
/**
  * @brief  : process the relink of eth
  * @param  : netif - - specify the ETH netif
  *           
  * @retval : none
  * @author : xuk
  */
void eth_re_link(struct netif *netif){
	
	__IO uint32_t tickstart = 0;
  uint32_t regvalue = 0, tmpreg = 0;
	if(netif_is_link_up(netif)){/*link up process*/
		
		if(ETH_InitStructure.ETH_AutoNegotiation == ETH_AutoNegotiation_Enable){/*AutoNegotiation_Enable*/
			
			/* Enable Auto-Negotiation */
      ETH_WritePHYRegister(DP83848_PHY_ADDRESS, PHY_BCR, PHY_AutoNegotiation);
			
			/* Wait until the auto-negotiation will be completed */
			do
			{
				tickstart++;
			} while (!(ETH_ReadPHYRegister(DP83848_PHY_ADDRESS, PHY_BSR) & PHY_AutoNego_Complete) && (tickstart < (uint32_t)PHY_READ_TO));
			
			/* Return ERROR in case of timeout */
			if(tickstart == PHY_READ_TO)
			{
//				return ETH_ERROR;
			}
			
			/* Reset Timeout counter */
			tickstart = 0;
			
			/* Read the result of the auto-negotiation */
			regvalue = ETH_ReadPHYRegister(DP83848_PHY_ADDRESS, PHY_SR);
		
			/* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
			if((regvalue & PHY_DUPLEX_STATUS) != (uint32_t)RESET)
			{
				/* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
				ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;  
			}
			else
			{
				/* Set Ethernet duplex mode to Half-duplex following the auto-negotiation */
				ETH_InitStructure.ETH_Mode = ETH_Mode_HalfDuplex;           
			}

			/* Configure the MAC with the speed fixed by the auto-negotiation process */
			if(regvalue & PHY_SPEED_STATUS)
			{  
				/* Set Ethernet speed to 10M following the auto-negotiation */    
				ETH_InitStructure.ETH_Speed = ETH_Speed_10M; 
			}
			else
			{   
				/* Set Ethernet speed to 100M following the auto-negotiation */ 
				ETH_InitStructure.ETH_Speed = ETH_Speed_100M;      
			}			
		}
		else{/*AutoNegotiation_Disable*/
			
			if(!ETH_WritePHYRegister(DP83848_PHY_ADDRESS, PHY_BCR, ((uint16_t)(ETH_InitStructure.ETH_Mode >> 3) |
                                                   (uint16_t)(ETH_InitStructure.ETH_Speed >> 1))))
			{
				/* Return ERROR in case of write timeout */
//				return ETH_ERROR;
			}
			/* Delay to assure PHY configuration */
//			_eth_delay_(PHY_CONFIG_DELAY);
			
		}
		
		
		
				/*------------------------ ETHERNET MACCR Configuration --------------------*/
		/* Get the ETHERNET MACCR value */  
		tmpreg = ETH->MACCR;
		/* Clear WD, PCE, PS, TE and RE bits */
		tmpreg &= MACCR_CLEAR_MASK;
		/* Set the WD bit according to ETH_Watchdog value */
		/* Set the JD: bit according to ETH_Jabber value */
		/* Set the IFG bit according to ETH_InterFrameGap value */ 
		/* Set the DCRS bit according to ETH_CarrierSense value */  
		/* Set the FES bit according to ETH_Speed value */ 
		/* Set the DO bit according to ETH_ReceiveOwn value */ 
		/* Set the LM bit according to ETH_LoopbackMode value */ 
		/* Set the DM bit according to ETH_Mode value */ 
		/* Set the IPCO bit according to ETH_ChecksumOffload value */                   
		/* Set the DR bit according to ETH_RetryTransmission value */ 
		/* Set the ACS bit according to ETH_AutomaticPadCRCStrip value */ 
		/* Set the BL bit according to ETH_BackOffLimit value */ 
		/* Set the DC bit according to ETH_DeferralCheck value */                          
		tmpreg |= (uint32_t)(ETH_InitStructure.ETH_Watchdog | 
										ETH_InitStructure.ETH_Jabber | 
										ETH_InitStructure.ETH_InterFrameGap |
										ETH_InitStructure.ETH_CarrierSense |
										ETH_InitStructure.ETH_Speed | 
										ETH_InitStructure.ETH_ReceiveOwn |
										ETH_InitStructure.ETH_LoopbackMode |
										ETH_InitStructure.ETH_Mode | 
										ETH_InitStructure.ETH_ChecksumOffload |    
										ETH_InitStructure.ETH_RetryTransmission | 
										ETH_InitStructure.ETH_AutomaticPadCRCStrip | 
										ETH_InitStructure.ETH_BackOffLimit | 
										ETH_InitStructure.ETH_DeferralCheck);
		/* Write to ETHERNET MACCR */
		ETH->MACCR = (uint32_t)tmpreg;
		
		/*----------------------- ETHERNET MACFFR Configuration --------------------*/ 
		/* Set the RA bit according to ETH_ReceiveAll value */
		/* Set the SAF and SAIF bits according to ETH_SourceAddrFilter value */
		/* Set the PCF bit according to ETH_PassControlFrames value */
		/* Set the DBF bit according to ETH_BroadcastFramesReception value */
		/* Set the DAIF bit according to ETH_DestinationAddrFilter value */
		/* Set the PR bit according to ETH_PromiscuousMode value */
		/* Set the PM, HMC and HPF bits according to ETH_MulticastFramesFilter value */
		/* Set the HUC and HPF bits according to ETH_UnicastFramesFilter value */
		/* Write to ETHERNET MACFFR */  
		ETH->MACFFR = (uint32_t)(ETH_InitStructure.ETH_ReceiveAll | 
														ETH_InitStructure.ETH_SourceAddrFilter |
														ETH_InitStructure.ETH_PassControlFrames |
														ETH_InitStructure.ETH_BroadcastFramesReception | 
														ETH_InitStructure.ETH_DestinationAddrFilter |
														ETH_InitStructure.ETH_PromiscuousMode |
														ETH_InitStructure.ETH_MulticastFramesFilter |
														ETH_InitStructure.ETH_UnicastFramesFilter); 
		/*--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------*/
		/* Write to ETHERNET MACHTHR */
		ETH->MACHTHR = (uint32_t)ETH_InitStructure.ETH_HashTableHigh;
		/* Write to ETHERNET MACHTLR */
		ETH->MACHTLR = (uint32_t)ETH_InitStructure.ETH_HashTableLow;
		/*----------------------- ETHERNET MACFCR Configuration --------------------*/
		/* Get the ETHERNET MACFCR value */  
		tmpreg = ETH->MACFCR;
		/* Clear xx bits */
		tmpreg &= MACFCR_CLEAR_MASK;
		
		/* Set the PT bit according to ETH_PauseTime value */
		/* Set the DZPQ bit according to ETH_ZeroQuantaPause value */
		/* Set the PLT bit according to ETH_PauseLowThreshold value */
		/* Set the UP bit according to ETH_UnicastPauseFrameDetect value */
		/* Set the RFE bit according to ETH_ReceiveFlowControl value */
		/* Set the TFE bit according to ETH_TransmitFlowControl value */  
		tmpreg |= (uint32_t)((ETH_InitStructure.ETH_PauseTime << 16) | 
										 ETH_InitStructure.ETH_ZeroQuantaPause |
										 ETH_InitStructure.ETH_PauseLowThreshold |
										 ETH_InitStructure.ETH_UnicastPauseFrameDetect | 
										 ETH_InitStructure.ETH_ReceiveFlowControl |
										 ETH_InitStructure.ETH_TransmitFlowControl); 
		/* Write to ETHERNET MACFCR */
		ETH->MACFCR = (uint32_t)tmpreg;
		/*----------------------- ETHERNET MACVLANTR Configuration -----------------*/
		/* Set the ETV bit according to ETH_VLANTagComparison value */
		/* Set the VL bit according to ETH_VLANTagIdentifier value */  
		ETH->MACVLANTR = (uint32_t)(ETH_InitStructure.ETH_VLANTagComparison | 
															 ETH_InitStructure.ETH_VLANTagIdentifier); 
				 
		/*-------------------------------- DMA Config ------------------------------*/
		/*----------------------- ETHERNET DMAOMR Configuration --------------------*/
		/* Get the ETHERNET DMAOMR value */  
		tmpreg = ETH->DMAOMR;
		/* Clear xx bits */
		tmpreg &= DMAOMR_CLEAR_MASK;
		
		/* Set the DT bit according to ETH_DropTCPIPChecksumErrorFrame value */
		/* Set the RSF bit according to ETH_ReceiveStoreForward value */
		/* Set the DFF bit according to ETH_FlushReceivedFrame value */
		/* Set the TSF bit according to ETH_TransmitStoreForward value */
		/* Set the TTC bit according to ETH_TransmitThresholdControl value */
		/* Set the FEF bit according to ETH_ForwardErrorFrames value */
		/* Set the FUF bit according to ETH_ForwardUndersizedGoodFrames value */
		/* Set the RTC bit according to ETH_ReceiveThresholdControl value */
		/* Set the OSF bit according to ETH_SecondFrameOperate value */
		tmpreg |= (uint32_t)(ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame | 
										ETH_InitStructure.ETH_ReceiveStoreForward |
										ETH_InitStructure.ETH_FlushReceivedFrame |
										ETH_InitStructure.ETH_TransmitStoreForward | 
										ETH_InitStructure.ETH_TransmitThresholdControl |
										ETH_InitStructure.ETH_ForwardErrorFrames |
										ETH_InitStructure.ETH_ForwardUndersizedGoodFrames |
										ETH_InitStructure.ETH_ReceiveThresholdControl |                                   
										ETH_InitStructure.ETH_SecondFrameOperate); 
		/* Write to ETHERNET DMAOMR */
		ETH->DMAOMR = (uint32_t)tmpreg;
		
		/*----------------------- ETHERNET DMABMR Configuration --------------------*/ 
		/* Set the AAL bit according to ETH_AddressAlignedBeats value */
		/* Set the FB bit according to ETH_FixedBurst value */
		/* Set the RPBL and 4*PBL bits according to ETH_RxDMABurstLength value */
		/* Set the PBL and 4*PBL bits according to ETH_TxDMABurstLength value */
		/* Set the DSL bit according to ETH_DesciptorSkipLength value */
		/* Set the PR and DA bits according to ETH_DMAArbitration value */         
		ETH->DMABMR = (uint32_t)(ETH_InitStructure.ETH_AddressAlignedBeats | 
														ETH_InitStructure.ETH_FixedBurst |
														ETH_InitStructure.ETH_RxDMABurstLength | /* !! if 4xPBL is selected for Tx or Rx it is applied for the other */
														ETH_InitStructure.ETH_TxDMABurstLength | 
													 (ETH_InitStructure.ETH_DescriptorSkipLength << 2) |
														ETH_InitStructure.ETH_DMAArbitration |
														ETH_DMABMR_USP); /* Enable use of separate PBL for Rx and Tx */
														
		#ifdef USE_ENHANCED_DMA_DESCRIPTORS
			/* Enable the Enhanced DMA descriptors */
			ETH->DMABMR |= ETH_DMABMR_EDE;
		#endif /* USE_ENHANCED_DMA_DESCRIPTORS */
																
		/* Return Ethernet configuration success */
//		return ETH_SUCCESS;
//		ETH_Start();
	}
	else{/*link down process*/
				
	}
}

