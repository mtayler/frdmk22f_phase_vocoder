/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    frdmk22f_phase_vocoder.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK22F51212.h"
#include "arm_math.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "fsl_sgtl5000.h"

/* TODO: insert other definitions and declarations here. */
#define BOARD_I2C_AC_ADDR (0b0001010)
#define BUFFER_SIZE (512U)
#define BUFFER_NUMBER (4U)

/* Function prototypes */
void applyEffect(uint16_t * rxBuffer, uint16_t * txBuffer, size_t bufferSize);

/* Variable */
extern codec_config_t boardCodecConfig;

AT_NONCACHEABLE_SECTION_ALIGN(static uint16_t rxBuffer[BUFFER_NUMBER*BUFFER_SIZE], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static uint16_t txBuffer[BUFFER_NUMBER*BUFFER_SIZE], 4);

static uint32_t tx_index = 0U, rx_index = 0U;

volatile uint32_t emptyBlock = BUFFER_NUMBER;

/*
 * @brief   Application entry point.
 */
int main(void) {
	codec_handle_t codecHandle = {0};
    sai_transfer_t xfer;

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();
    /* Init audio codec */
    BOARD_InitAUDIOPeripheral();
    /* Init codec */
    CODEC_Init(&codecHandle, &boardCodecConfig);
    CODEC_SetFormat(&codecHandle, BOARD_SAI_AC_RX_MCLK_SOURCE_CLOCK_HZ, BOARD_SAI_AC_rx_format.sampleRate_Hz, BOARD_SAI_AC_rx_format.bitWidth);
    SGTL_SetVolume(&codecHandle, kSGTL_ModuleHP, 0x20);

    while(1)
    {
    	if(emptyBlock > 0)
    	{
    		xfer.data = (uint8_t *)(rxBuffer + rx_index * BUFFER_SIZE);
    		xfer.dataSize = BUFFER_SIZE;
    		if(kStatus_Success == SAI_TransferReceiveEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_rxHandle, &xfer))
    		{
    			rx_index++;
    		}
    		if(rx_index == BUFFER_NUMBER)
    		{
    			rx_index = 0U;
    		}
    	}
    	if(emptyBlock < BUFFER_NUMBER)
    	{
    		// If queue not full
    		if (!BOARD_eDMA_AC_txHandle.saiQueue[BOARD_eDMA_AC_txHandle.queueUser].data) {
        		// Copy from rxBuffer to txBuffer
    			arm_copy_q15(rxBuffer + tx_index * BUFFER_SIZE, txBuffer + tx_index * BUFFER_SIZE, BUFFER_SIZE);
    			xfer.data = (uint8_t *)(txBuffer + tx_index * BUFFER_SIZE);	//
    			xfer.dataSize = BUFFER_SIZE;
    			if(kStatus_Success == SAI_TransferSendEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_txHandle, &xfer))
    			{
    				tx_index++;
    			}
    			if(tx_index == BUFFER_NUMBER)
    			{
    				tx_index = 0U;
    			}
    		}
    	}
    }
}

void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    if(kStatus_SAI_RxError == status)
    {
        /* Handle the error. */
    }
    else
    {
        emptyBlock--;
    }
}

void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
    if(kStatus_SAI_TxError == status)
    {
        /* Handle the error. */
    }
    else
    {
        emptyBlock++;
    }
}
