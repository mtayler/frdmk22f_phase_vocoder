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
#define BUFFER_SIZE (1024U)
#define BUFFER_NUMBER (4U)

/* Function prototypes */
void applyEffect(uint16_t * rxBuffer, uint16_t * txBuffer, size_t bufferSize);

/* Variable */
extern codec_config_t boardCodecConfig;

AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t Buffer[BUFFER_NUMBER*BUFFER_SIZE], 4);

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

    // Setup effects

    CODEC_Init(&codecHandle, &boardCodecConfig);
    CODEC_SetFormat(&codecHandle, BOARD_SAI_AC_tx_format.masterClockHz, BOARD_SAI_AC_tx_format.sampleRate_Hz, BOARD_SAI_AC_tx_format.bitWidth);

//    SGTL_EnableModule(&codecHandle, kSGTL_ModuleHP);
//    SGTL_EnableModule(&codecHandle, kSGTL_ModuleLineIn);
//    SGTL_EnableModule(&codecHandle, kSGTL_ModuleI2SIN);
//    SGTL_EnableModule(&codecHandle, kSGTL_ModuleI2SOUT);
//    SGTL_SetMute(&codecHandle, kSGTL_ModuleHP, false);
//    SGTL_SetMute(&codecHandle, kSGTL_ModuleLineIn, false);
//    SGTL_SetMute(&codecHandle, kSGTL_ModuleI2SIN, false);
//    SGTL_SetMute(&codecHandle, kSGTL_ModuleI2SOUT, false);
    SGTL_SetVolume(&codecHandle, kSGTL_ModuleHP, 0x30);
    SGTL_SetVolume(&codecHandle, kSGTL_ModuleLineIn, 0x00);
    SGTL_SetVolume(&codecHandle, kSGTL_ModuleI2SIN, 0x00);
    SGTL_SetVolume(&codecHandle, kSGTL_ModuleI2SOUT, 0x00);

    xfer.dataSize = BUFFER_SIZE;

//    // Start as many receive as capable
//	for (rx_index=0; rx_index < BUFFER_NUMBER; rx_index++) {
//		if (rx_index == BUFFER_NUMBER) {
//			rx_index = 0;
//		}
//		rxfer.data = (uint8_t *)rxBuffer + rx_index * BUFFER_SIZE;
//		// If we run out of queue room, stop trying
//		if (SAI_TransferReceiveEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_rxHandle, &rxfer) != kStatus_Success) {
//			break;
//		}
//	}

    while(1)
    {
    	if(emptyBlock > 0)
    	{
    		xfer.data = Buffer + rx_index * BUFFER_SIZE;
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
    		xfer.data = Buffer + tx_index * BUFFER_SIZE;
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

//void applyEffect(uint16_t * rxBuffer, uint16_t * txBuffer, size_t bufferSize) {
//	// Compute solely from previous samples
//	arm_mean_q15(previousSamples, SAMPLES, txBuffer);
//	// Compute samples using both previousSamples and Buffer
//	for (size_t i=1; i < SAMPLES; i++) {
//		uint16_t temp = 0;
//		size_t j;
//		for (j=i; j < SAMPLES; j++) {
//			temp += previousSamples[j]/SAMPLES;
//		}
//		for (; j < i; j++) {
//			temp += rxBuffer[j-SAMPLES]/SAMPLES;
//		}
//		txBuffer[SAMPLES-j] = temp;
//		previousSamples[i] = rxBuffer[BUFFER_SIZE-SAMPLES+i];
//	}
//	for (size_t i=SAMPLES; i < BUFFER_SIZE - SAMPLES; i++) {
//		arm_mean_q15(&rxBuffer[i-SAMPLES], SAMPLES, &txBuffer[i-SAMPLES]);
//	}
//}


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
