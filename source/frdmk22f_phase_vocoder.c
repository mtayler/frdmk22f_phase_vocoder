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
#include <cr_section_macros.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK22F51212.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include <math.h>
#include "arm_math.h"
#include "fsl_sgtl5000.h"

/* TODO: insert other definitions and declarations here. */
#define BOARD_I2C_AC_ADDR (0b0001010)
#define BUFFER_SIZE (4096U)
#define BUFFER_NUMBER (4U)
#define FFT_BUFFER_SIZE (BUFFER_SIZE*2)

#define ARM_FFT (0)
#define ARM_IFFT (1)

#define ARM_BIT_NORMAL (0)
#define ARM_BIT_REVERSE (1)

/* Function prototypes */
void main(void) __SECTION(text, SRAM_UPPER);
void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData) __RAMFUNC(SRAM_UPPER);

/* Variable */
extern codec_config_t boardCodecConfig;

// Data buffers
AT_NONCACHEABLE_SECTION_ALIGN(static int16_t rxBuffer[BUFFER_NUMBER*BUFFER_SIZE], 4) __BSS(SRAM_LOWER);
AT_NONCACHEABLE_SECTION_ALIGN(static int16_t txBuffer[BUFFER_NUMBER*BUFFER_SIZE], 4) __BSS(SRAM_LOWER);
AT_NONCACHEABLE_SECTION_ALIGN(static float32_t fftBuffer[FFT_BUFFER_SIZE], 4) __BSS(SRAM_UPPER);

volatile int16_t * receivedData = NULL;

/*
 * @brief   Application entry point.
 */
void main(void) {
	codec_handle_t codecHandle = {0};
	sai_transfer_t xfer;
	arm_rfft_fast_instance_f32 fftInst;
	volatile bool done = false;
	int16_t * receivedBuffer;	// Used to save receivedData to prevent it changing while being used

	size_t tx_index = 0;

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

    // Init RFFT instance
    arm_rfft_fast_init_f32(&fftInst, BUFFER_SIZE);

    // Start as many receives as we can
	xfer.dataSize = BUFFER_SIZE*sizeof(int16_t);	// Size in bytes
	// Don't wrap rx_index here because we'll only go until we reach buffer size, and deal with it later
    for (size_t rx_index=0; rx_index < BUFFER_NUMBER; rx_index++) {
    	xfer.data = (uint8_t *)&rxBuffer[rx_index*BUFFER_SIZE];
		BOARD_eDMA_AC_rxHandle.userData = (int16_t *)xfer.data;
    	if (SAI_TransferReceiveEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_rxHandle, &xfer) == kStatus_SAI_QueueFull) {
    		// If the receive queue is full, we've started as many as we can
    		break;
    	}
    }

	while (!done) {
		// Wait for received data
		while (receivedData == NULL);
		// Do processing and add to transmit buffer
		receivedBuffer = receivedData;
		receivedData = NULL;

		// Convert from uint16_t to float going from rxBuffer to fftBuffer
		arm_q15_to_float((q15_t *)receivedBuffer, fftBuffer, BUFFER_SIZE);
//		arm_copy_q15(receivedBuffer, &txBuffer[tx_index], BUFFER_SIZE);

		// Done with receive buffer, queue another (finishes sequentially)
		xfer.data = (uint8_t *)receivedBuffer;
		xfer.dataSize = BUFFER_SIZE*sizeof(int16_t);	// Size in bytes
		BOARD_eDMA_AC_rxHandle.userData = xfer.data;

		SAI_TransferReceiveEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_rxHandle, &xfer);
		// FFT in-place
		arm_rfft_fast_f32(&fftInst, fftBuffer, fftBuffer, ARM_FFT);
		// IFFT in-place
		arm_rfft_fast_f32(&fftInst, fftBuffer, fftBuffer, ARM_IFFT);
		// Float to q15 from fftBuffer to txBuffer;
		arm_float_to_q15(fftBuffer, (q15_t *)&txBuffer[tx_index*BUFFER_SIZE], BUFFER_SIZE);

		// Put processed data on send queue
		xfer.data = (uint8_t *)&txBuffer[tx_index*BUFFER_SIZE];
		xfer.dataSize = BUFFER_SIZE*sizeof(int16_t);	// Size in bytes

		/* Loop until queue has space */
		while(SAI_TransferSendEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_txHandle, &xfer) == kStatus_SAI_QueueFull);
		tx_index += (tx_index + 1) % BUFFER_NUMBER;
	}
}


void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
//	static Message_data msg;
	assert(status != kStatus_SAI_RxError);
	// TODO: Need to deal with wrapping circular buffer here
	/* Message queues too slow */
//	msg.data = (int16_t *)handle->saiQueue[handle->queueDriver].data;
//	msg.dataSize = handle->transferSize[handle->queueDriver];
//	if (! xQueueSendToBackFromISR(processQueue, &msg, NULL)) {
//		// Queue was full
//		configASSERT(pdFALSE);
//	}
	receivedData = (volatile int16_t *)userData;
}

//void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
//{
//	if (status == kStatus_SAI_TxError) {
//		// deal with error
//	}
//	// Tell send task there is more room on send queue
//	vTaskNotifyGiveFromISR(taskHandle_sendData, NULL);
//}
