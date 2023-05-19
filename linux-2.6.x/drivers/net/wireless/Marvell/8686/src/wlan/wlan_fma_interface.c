/** @file wlan_fma_interface.c
  *  
  * @brief This file contains functions for FMA interface.
  * 
  */
/** 
  * @section copyright_sec Copyright
  *
  * (c) Copyright © 2007, Motorola.
  *
  * This file is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * at your option) any later version.
  *
  * This file is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * Revision History:
  * Author           Date            Description
  * Motorola         20-Oct-2007     Initial creation for FMA interface
  */

#include    "include.h"
#include    "fma_interface.h"

#define IPTOS_OFFSET 5

#define MAX_CONTEND_TYPES   3
#define MAX_NO_OF_REGISTERS 2

#define NUM_OF_RANK_LEVELS_TWO  2
#define ARB_RANK_ZERO 0

/* FMA precendece values */
enum
{
  WLAN_FMA_PREC_VO=0,
  WLAN_FMA_PREC_VI,
  WLAN_FMA_PREC_BE,
  WLAN_FMA_PREC_BK
};

/* BCA Register table, used to update the arbitration rank */
struct
{
  u32 reg_addr;
  u32 reg_value[MAX_CONTEND_TYPES];
}wlan_arb_reg_values[MAX_NO_OF_REGISTERS] = {
   { 0xa5dc,               /* WLAN Tx ok register */
             { 0xffff0000, /* NO_CONTENTION */
               0xffff0000, /* WLAN_CONTENTION */
               0xffff0f0f  /* BT_CONTENTION */
             }
   },
   { 0xa5e0,               /* BT Tx ok register */
            { 0x0000ffff,  /* NO_CONTENTION */
              0x0000ffff,  /* WLAN_CONTENTION */
              0x0000f0f0   /* BT_CONTENTION */
            }
   }
};

/********************************************************
                Local Variables
********************************************************/

/* Table to convert the packet TOS to FMA precedence */
static fma_precedence_t tos2prec[8] = {
    WLAN_FMA_PREC_BE, // for AC_PRIO_BE
    WLAN_FMA_PREC_BK, // for AC_PRIO_BK
    WLAN_FMA_PREC_BK, // for AC_PRIO_BK
    WLAN_FMA_PREC_BE, // for AC_PRIO_BE
    WLAN_FMA_PREC_VI, // for AC_PRIO_VI
    WLAN_FMA_PREC_VI, // for AC_PRIO_VI
    WLAN_FMA_PREC_VO, // for AC_PRIO_VO
    WLAN_FMA_PREC_VO  // for AC_PRIO_VO
};

/* callbacks to register with FMA */
static void wlan_send_traffic_input(u8 send_precedence);
static int wlan_set_arb_rank(fma_arb_decision_t arb_decision);

static fma_precedence_t wlan_get_prec(struct sk_buff *skb);
static int wlan_set_arb_reg(u32 regoffset, u32 regvalue);

/* Driver information to register with FMA */
static fma_driver_info_t driver_info = { 
                    "WLAN",  /* Driver name */
                    {0,0},   /* Driver operating freq range (for future use only) */  
                   &wlan_send_traffic_input,
                   &wlan_set_arb_rank,
                    NULL };  /* callback to trigger frequency change (for future use only) */

/* Place to hold the driver handle obtained from FMA */
static u32 driver_handle = 0;

/* Tells whether we need to send traffic inputs or not */
static u32 send_traffic_input = 0;

/* Used to store the previous precedence info */
static fma_precedence_t prev_prec = WLAN_FMA_PREC_BE ;

/********************************************************
                Local Functions
********************************************************/

/** 
 *  @brief This function gets the packet precedence from the TOS
 *  
 *  @param skb		A pointer to skb which includes TX packet
 *  @return 		precedence level
 */
static fma_precedence_t wlan_get_prec(struct sk_buff *skb)
{
    u8 tos;
    fma_precedence_t prec;
    struct ethhdr *eth = (struct ethhdr *) skb->data;

    switch (eth->h_proto) {
    case __constant_htons(ETH_P_IP):
        PRINTM(DATA, "WLAN_FMA: packet type ETH_P_IP: %04x, tos=%#x prio=%#x\n",
               eth->h_proto, skb->nh.iph->tos, skb->priority);
        tos = IPTOS_PREC(skb->nh.iph->tos) >> IPTOS_OFFSET;
        break;
    case __constant_htons(ETH_P_ARP):
        PRINTM(DATA, "WLAN_FMA: ARP packet %04x\n", eth->h_proto);
    default:
        tos = 0;
        break;
    }
    if (tos >= NELEMENTS(tos2prec)) {
      return WLAN_FMA_PREC_BE;
    }

    prec = tos2prec[tos];
    return prec;
}

/** 
 *  @brief This function is a callback handler to record whether WLAN driver 
 *         has to send traffic indications or not
 *  
 *  @param send_precedence	Notification from FMA whether to send traffic input or not
 *  @return 			None
 */
static void wlan_send_traffic_input(u8 send_precedence)
{
  PRINTM(INFO, "WLAN_FMA: FMA traffic request %d\n", send_precedence);

  send_traffic_input = send_precedence;
}

/** 
 *  @brief This function is a call back handler to set the BCA register's as per the FMA rank
 *  
 *  @param arb_decision		FMA arbitration decision
 *  @return 			WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int wlan_set_arb_rank(fma_arb_decision_t arb_decision)
{
  int ret = WLAN_STATUS_SUCCESS;
  int cnt;

  /* Set the respective register values according to the tx_rank */

  if(arb_decision.num_of_tx_devices == NUM_OF_RANK_LEVELS_TWO) {
    /* FMA's arbitration decision is either of WLAN_LP > BT_LP or BT_LP > WLAN_LP */

    if( arb_decision.tx_rank[ARB_RANK_ZERO] < MAX_CONTEND_TYPES ) {
      /* Arbitration decision is either of the known contending devices */

      /* Set the respective value for each of the BCA register, as per the rank */
      for(cnt=0; cnt<MAX_NO_OF_REGISTERS; cnt++) {
        ret = wlan_set_arb_reg( wlan_arb_reg_values[cnt].reg_addr, 
                      wlan_arb_reg_values[cnt].reg_value[arb_decision.tx_rank[ARB_RANK_ZERO]]);
        if(ret != WLAN_STATUS_SUCCESS) {
          PRINTM(ERROR, "WLAN_FMA: wlan_set_arb_reg failed with %d\n", ret);
          break;
        }
      }
    }
    else {
      PRINTM(ERROR, "WLAN_FMA: invalid arbitration rank %d\n", arb_decision.tx_rank[ARB_RANK_ZERO]);
    }
  }
  else {
    PRINTM(ERROR, "WLAN_FMA: invalid number of Tx devices  %d\n", arb_decision.num_of_tx_devices);
  }

  return ret;
}

/** 
 *  @brief This function prepares and sends a command to firmware to set the 
 *         arbitration register
 *  
 *  @param regoffset	Offset of the arbitration register to be set
 *  @param regvalue 	Value of the arbitration register to be set
 *  @return 		WLAN_STATUS_SUCCESS or WLAN_STATUS_FAILURE
 */
static int wlan_set_arb_reg(u32 regoffset, u32 regvalue)
{
    wlan_private      *priv = wlanpriv;  /* wlanpriv is a global variable */
    wlan_offset_value  offval;
    int                ret  = WLAN_STATUS_SUCCESS;

    offval.offset = regoffset;
    offval.value  = regvalue;

    PRINTM(INFO, "WLAN_FMA: Offset: %08x Value: %08x\n", offval.offset, offval.value);
    
    ret = PrepareAndSendCommand(priv, HostCmd_CMD_MAC_REG_ACCESS,
                                HostCmd_ACT_GEN_WRITE, 0, 0, &offval);

    return ret;
}

/********************************************************
                Global Functions
********************************************************/

/** 
 *  @brief This function gets the the traffic precedence from TOS and if it has 
 *         changed, sends an indication to FMA
 *  
 *  @param skb		A pointer to skb which includes TX packet
 *  @return 		None
 */
void wlan_notify_fma(struct sk_buff *skb)
{
    int ret = 0;
    fma_precedence_t prec = 0;

    /* Get the precedence */
    prec = wlan_get_prec(skb);

    PRINTM(INFO, "WLAN_FMA: Sending packet=<%p> out, prev_prec=<%d> new_prec=<%d>\n",
            skb, prev_prec, prec);

    if (send_traffic_input == 1 && prec != prev_prec) {
        ret = FMA_traffic_precedence_indication(driver_handle, prec);
        if (ret < 0) {
            PRINTM(ERROR, "WLAN_FMA: traffic_pre_ind failed, err=%d\n", ret);
        }
        else {
            PRINTM(INFO, "WLAN_FMA: traffic_pre_ind success\n");
        }

        prev_prec = prec;
    }
}

/** 
 *  @brief This function registers with FMA and obtains a driver handle for 
 *         further communication
 *  
 *  @return 		WLAN_STATUS_SUCCESS
 */
int wlan_fma_register(void)
{
    int ret = WLAN_STATUS_SUCCESS;

    /* If already registered, return success */
    if(driver_handle)
       return WLAN_STATUS_SUCCESS;

    ret = FMA_register_driver(&driver_info, &driver_handle);
    if (ret < 0) {
        PRINTM(ERROR, "WLAN_FMA: FMA register failed, err=%d\n", ret);

        /* If ret == -ENOMEM or ret == -EAGAIN, we need to try again to register */
    }
    else {
        PRINTM(INFO, "WLAN_FMA: FMA register success, handle=0x%x\n", driver_handle);
    }

    return WLAN_STATUS_SUCCESS;
}

/** 
 *  @brief This function checks if we have registered with FMA or not
 *  
 *  @return 		TRUE or FALSE
 */
BOOLEAN wlan_is_fma_registered(void)
{
    return (driver_handle) ? TRUE : FALSE;
}

/** 
 *  @brief This function un-registers from FMA
 *  
 *  @return 		None
 */
void wlan_fma_unregister(void)
{
    int ret = 0;
    if (driver_handle) {
       ret = FMA_unregister_driver(driver_handle);
       if (ret < 0) {
           PRINTM(ERROR, "WLAN_FMA: unregister failed, err=%d\n", ret);
       }
       else {
           PRINTM(INFO, "WLAN_FMA: unregister success\n");
       }

       send_traffic_input = 0;
       driver_handle = 0;
    }
}
