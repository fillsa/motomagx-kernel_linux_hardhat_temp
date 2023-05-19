#ifndef _WLAN_FMA_INTERFACE_H_
#define _WLAN_FMA_INTERFACE_H_

/** @file wlan_fma_interface.h
  *  
  * @brief This is header file for FMA interface APIs.
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

/* Function to Register with FMA */
extern int wlan_fma_register(void);

/* Function to Un-register with FMA */
extern void wlan_fma_unregister(void);

/* Function to check if we have already registered with FMA */
extern BOOLEAN wlan_is_fma_registered(void);

/* Function to send traffic precedence input to FMA */
extern void wlan_notify_fma(struct sk_buff *skb);

#endif /* _WLAN_FMA_INTERFACE_H_ */
