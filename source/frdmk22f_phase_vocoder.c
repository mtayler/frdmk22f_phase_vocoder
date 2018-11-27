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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "fsl_sgtl5000.h"

/* TODO: insert other definitions and declarations here. */
#define BOARD_I2C_AC_ADDR (0b0001010)
#define BUFFER_SIZE (2048U)
#define BUFFER_NUMBER (4U)

#define ARM_FFT (0)
#define ARM_IFFT (1)

#define ARM_BIT_NORMAL (1)
#define ARM_BIT_REVERSE (0)

#define PROCESSING_QUEUE_LENGTH (SAI_XFER_QUEUE_SIZE)
#define SEND_QUEUE_LENGTH (SAI_XFER_QUEUE_SIZE)	// should only ever get as many requests as receive queues

#define TASK_PROCESSDATA_PRIORITY (configMAX_PRIORITIES)
#define TASK_SENDDATA_PRIORITY (configMAX_PRIORITIES)

/* Function prototypes */
void task_processData(void *pvParameters);
void task_sendData(void *pvParameters);

/* Variable */
extern codec_config_t boardCodecConfig;

//TaskHandle_t taskHandle_sendData;

volatile int16_t * receivedData = NULL;

typedef struct _Message_data {
	uint8_t messageID;
	int16_t * data;
	size_t index;
	size_t dataSize;
	/* Circular buffer somehow? */
} Message_data;

/*
 * @brief   Application entry point.
 */
int main(void) {
	codec_handle_t codecHandle = {0};

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

    // Start tasks
    xTaskCreate(task_processData, "task_processData", configMINIMAL_STACK_SIZE, NULL, TASK_PROCESSDATA_PRIORITY, NULL);
//    xTaskCreate(task_sendData, "task_sendData", configMINIMAL_STACK_SIZE, NULL, TASK_SENDDATA_PRIORITY, &taskHandle_sendData);

    // Notify send task all send queues free queues
//    xTaskNotify(taskHandle_sendData, SAI_XFER_QUEUE_SIZE, eSetValueWithOverwrite);

    // Make sure we can call ISR syscalls from DMA callbacks
    NVIC_SetPriority(DMA0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(DMA1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    vTaskStartScheduler();

    configASSERT(pdFALSE);	// shouldn't get here
    while (1) {
    }
}

void task_processData(void *pvParameters) {
//	Message_data msg;
    arm_rfft_instance_q15 fftInst;
	sai_transfer_t xfer;
	int16_t * processData;

	AT_NONCACHEABLE_SECTION_ALIGN(static int16_t rxBuffer[BUFFER_NUMBER*BUFFER_SIZE], 4);
	AT_NONCACHEABLE_SECTION_ALIGN(static int16_t txBuffer[BUFFER_NUMBER*BUFFER_SIZE], 4);

	size_t rx_index = 0;
	size_t tx_index = 0;

    // Init RFFT instance
    arm_rfft_init_q15(&fftInst, BUFFER_SIZE, ARM_FFT, ARM_BIT_NORMAL);

//	// Configure queue
//	processQueue = xQueueCreate(PROCESSING_QUEUE_LENGTH, sizeof(Message_data));
//	configASSERT(processQueue != NULL);
//	vQueueAddToRegistry(sendQueue, "sendQueue");

    // Start as many receives as we can
	xfer.dataSize = BUFFER_SIZE;
    for (rx_index=0; rx_index < BUFFER_SIZE * BUFFER_NUMBER && rx_index/BUFFER_SIZE < SAI_XFER_QUEUE_SIZE; rx_index += BUFFER_SIZE) {
    	xfer.data = (uint8_t *)&rxBuffer[rx_index];
    	SAI_TransferReceiveEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_rxHandle, &xfer);
    }
	if (rx_index == BUFFER_SIZE * BUFFER_NUMBER) {
		rx_index = 0;
	}

	while (pdTRUE) {
		// Wait for received data
		while (receivedData == NULL);
		// Save value
		processData = receivedData;

		// Do processing and add to transmit buffer
		// FFT from rxBuffer to txBuffer
		fftInst.ifftFlagR = ARM_FFT;
//		arm_rfft_q15(&fftInst, processData, &txBuffer[tx_index]);
		arm_copy_q15(processData, &txBuffer[tx_index], BUFFER_SIZE);

		// Done with receive buffer, queue another
		xfer.data = (uint8_t *)&rxBuffer[rx_index];
		xfer.dataSize = BUFFER_SIZE;
		rx_index += BUFFER_SIZE;
		if (rx_index == BUFFER_SIZE * BUFFER_NUMBER) {
			rx_index = 0;
		}
		SAI_TransferReceiveEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_rxHandle, &xfer);

		/* Reset receivedData */
		receivedData = NULL;

		// Do IFFT in-place
//		arm_rfft_q15(&fftInst, &txBuffer[tx_index], &txBuffer[tx_index]);

		// Put processed data on send queue
		xfer.data = (uint8_t *)&txBuffer[tx_index];
		xfer.dataSize = BUFFER_SIZE;
		/* Loop until queue has space */
		while(SAI_TransferSendEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_txHandle, &xfer) == kStatus_SAI_QueueFull);
		tx_index += BUFFER_SIZE;
		if (tx_index == BUFFER_SIZE * BUFFER_NUMBER) {
			tx_index = 0;
		}
	}
}

void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
//	static Message_data msg;
	if (status == kStatus_SAI_RxError) {
		// deal with error
	}
	// TODO: Need to deal with wrapping circular buffer here
	/* Message queues too slow */
//	msg.data = (int16_t *)handle->saiQueue[handle->queueDriver].data;
//	msg.dataSize = handle->transferSize[handle->queueDriver];
//	if (! xQueueSendToBackFromISR(processQueue, &msg, NULL)) {
//		// Queue was full
//		configASSERT(pdFALSE);
//	}
	receivedData = (int16_t *)handle->saiQueue[handle->queueDriver].data;
}

//void txCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
//{
//	if (status == kStatus_SAI_TxError) {
//		// deal with error
//	}
//	// Tell send task there is more room on send queue
//	vTaskNotifyGiveFromISR(taskHandle_sendData, NULL);
//}
