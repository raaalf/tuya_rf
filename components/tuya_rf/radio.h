/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, CMOSTEK SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * Copyright (C) CMOSTEK SZ.
 */

/*!
 * @file    radio.h
 * @brief   Generic radio handlers
 *
 * @version 1.2
 * @date    Jul 17 2017
 * @author  CMOSTEK R@D
 */
 
#ifndef __RADIO_H
#define __RADIO_H

#include "cmt2300a.h"
#include <stdint.h>

#define TUYA_RF_TX_PROFILE_868_RFPDK 0
#define TUYA_RF_TX_PROFILE_868_AHOY_OPENDTU 1

#ifdef __cplusplus 
extern "C" { 
#endif

int RF_Init(void);
int RF_SetFrequency(uint16_t frequency_mhz);
int StartTx(uint16_t frequency_mhz, uint8_t tx_profile_868, int8_t tx_power_868_dbm);
int StartRx(uint16_t frequency_mhz);

#ifdef __cplusplus 
} 
#endif

#endif
