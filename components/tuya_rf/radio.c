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
 * @file    radio.c
 * @brief   Generic radio handlers
 *
 * @version 1.2
 * @date    Jul 17 2017
 * @author  CMOSTEK R@D
 */

#include "radio.h"
#include "cmt2300a_hal.h"
#include "cmt2300a_params_captured.h"
#include <stddef.h>

static uint16_t g_current_frequency_mhz = 433;
static uint16_t g_current_tx_bank_frequency_mhz = 433;

static const uint8_t *RF_GetFrequencyBank(uint16_t frequency_mhz)
{
    if (frequency_mhz == 315) {
        return g_cmt2300aFrequencyBank315;
    }
    if (frequency_mhz == 433) {
        return g_cmt2300aFrequencyBank;
    }
    if (frequency_mhz == 868) {
        return g_cmt2300aFrequencyBank868;
    }
    return NULL;
}

static int RF_ConfigFrequencyBank(uint16_t frequency_mhz)
{
    const uint8_t *frequency_bank = RF_GetFrequencyBank(frequency_mhz);
    if (frequency_bank == NULL) {
        return 3;
    }
    if (g_current_frequency_mhz == frequency_mhz) {
        return 0;
    }
    if (!CMT2300A_ConfigRegBank(CMT2300A_FREQUENCY_BANK_ADDR, frequency_bank, CMT2300A_FREQUENCY_BANK_SIZE)) {
        return 1;
    }
    g_current_frequency_mhz = frequency_mhz;
    return 0;
}

static int RF_ConfigTxBank(uint16_t frequency_mhz)
{
    const uint8_t *tx_bank = g_cmt2300aTxBank;
    uint16_t tx_bank_frequency_mhz = 433;
    if (frequency_mhz == 868) {
        tx_bank = g_cmt2300aTxBank868;
        tx_bank_frequency_mhz = 868;
    }
    if (g_current_tx_bank_frequency_mhz == tx_bank_frequency_mhz) {
        return 0;
    }
    if (!CMT2300A_ConfigRegBank(CMT2300A_TX_BANK_ADDR, tx_bank, CMT2300A_TX_BANK_SIZE)) {
        return 1;
    }
    g_current_tx_bank_frequency_mhz = tx_bank_frequency_mhz;
    return 0;
}

int RF_SetFrequency(uint16_t frequency_mhz)
{
    if (!CMT2300A_GoStby()) {
        return 2;
    }
    return RF_ConfigFrequencyBank(frequency_mhz);
}

int RF_Init(void)
{
    uint8_t tmp;
    
    CMT2300A_InitGpio();
	CMT2300A_Init();
    
    /* Config registers */
    CMT2300A_ConfigRegBank(CMT2300A_CMT_BANK_ADDR       , g_cmt2300aCmtBank       , CMT2300A_CMT_BANK_SIZE       );
    CMT2300A_ConfigRegBank(CMT2300A_SYSTEM_BANK_ADDR    , g_cmt2300aSystemBank    , CMT2300A_SYSTEM_BANK_SIZE    );
    CMT2300A_ConfigRegBank(CMT2300A_FREQUENCY_BANK_ADDR , g_cmt2300aFrequencyBank , CMT2300A_FREQUENCY_BANK_SIZE );
    CMT2300A_ConfigRegBank(CMT2300A_DATA_RATE_BANK_ADDR , g_cmt2300aDataRateBank  , CMT2300A_DATA_RATE_BANK_SIZE );
    CMT2300A_ConfigRegBank(CMT2300A_BASEBAND_BANK_ADDR  , g_cmt2300aBasebandBank  , CMT2300A_BASEBAND_BANK_SIZE  );
    CMT2300A_ConfigRegBank(CMT2300A_TX_BANK_ADDR        , g_cmt2300aTxBank        , CMT2300A_TX_BANK_SIZE        );
    g_current_frequency_mhz = 433;
    g_current_tx_bank_frequency_mhz = 433;
    
    // xosc_aac_code[2:0] = 2
    tmp = (~0x07) & CMT2300A_ReadReg(CMT2300A_CUS_CMT10);
    CMT2300A_WriteReg(CMT2300A_CUS_CMT10, tmp|0x02);
    
	if(false==CMT2300A_IsExist()) 
	{
        //CMT2300A not found!
        return -1;
    }
    else
	{
        return 0;
    }
}

int StartTx(uint16_t frequency_mhz) {
    if (!CMT2300A_GoStby()) {
        return 2;
    }
    int freq_res = RF_ConfigFrequencyBank(frequency_mhz);
    if (freq_res != 0) {
        return freq_res;
    }
    int tx_bank_res = RF_ConfigTxBank(frequency_mhz);
    if (tx_bank_res != 0) {
        return tx_bank_res;
    }
    CMT2300A_WriteReg(CMT2300A_CUS_SYS2,0);
    CMT2300A_ConfigGpio(CMT2300A_GPIO1_SEL_DOUT | CMT2300A_GPIO3_SEL_DIN | CMT2300A_GPIO2_SEL_INT2);
	CMT2300A_EnableTxDin(true);
	CMT2300A_ConfigTxDin(CMT2300A_TX_DIN_SEL_GPIO1);
	CMT2300A_EnableTxDinInvert(false);
	CMT2300A_ClearInterruptFlags();  // Clear stale TX_DONE flags to prevent false early exit in AutoSwitchStatus
	delay(2);  // Stabilization delay: let CMT2300A settle in Standby before TX transition
	if (CMT2300A_GoTx()) {
	    delay(2);  // Stabilization delay: let CMT2300A fully enter TX mode
        return 0;
    } else {
        return 2;
    }
}
 

int StartRx(uint16_t frequency_mhz) {
    if (!CMT2300A_GoStby()) {
        return 2;
    }
    int freq_res = RF_ConfigFrequencyBank(frequency_mhz);
    if (freq_res != 0) {
        return freq_res;
    }

	CMT2300A_WriteReg(CMT2300A_CUS_SYS2 , 0);
	CMT2300A_EnableTxDin(false);
	CMT2300A_EnableFifoMerge(true);
	CMT2300A_WriteReg(CMT2300A_CUS_PKT29, 0x20); 
	CMT2300A_ConfigGpio (CMT2300A_GPIO1_SEL_DCLK | CMT2300A_GPIO2_SEL_DOUT | CMT2300A_GPIO3_SEL_INT2);
	CMT2300A_ConfigInterrupt(CMT2300A_INT_SEL_SYNC_OK, CMT2300A_INT_SEL_SL_TMO);
	CMT2300A_EnableInterrupt(0);
	CMT2300A_ClearInterruptFlags();
	CMT2300A_ClearRxFifo();
	delay(2);  // Stabilization delay: let FIFO clear complete before RX transition

    if (!CMT2300A_GoRx()) {
        return 2;
    }

	CMT2300A_ClearInterruptFlags();
    return 0;
}
