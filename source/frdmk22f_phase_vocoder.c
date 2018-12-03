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
#include <stdint.h>
#include <math.h>
#include <cr_section_macros.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK22F51212.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "arm_math.h"
#include "fsl_sgtl5000.h"

/* TODO: insert other definitions and declarations here. */
#define BOARD_I2C_AC_ADDR (0b0001010)
#define BUFFER_SIZE (4096U)
#define BUFFER_NUMBER (4U)
#define FFT_SIZE BUFFER_SIZE >> 1U
#define FFT_BUFFER_SIZE (BUFFER_SIZE*2)

#define M_PI (3.14159265359f)

#define ARM_FFT (0)
#define ARM_IFFT (1)

#define ARM_BIT_NORMAL (0)
#define ARM_BIT_REVERSE (1)

/* Function prototypes */
void main(void) __SECTION(text, SRAM_UPPER);
__STATIC_INLINE void effectWhisper(float32_t * fftBuffer);
__STATIC_INLINE void effectRobot(float32_t * fftBuffer);
void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData) __RAMFUNC(SRAM_UPPER);
__STATIC_INLINE void int16_packed_to_float(int16_t * pSrc, float32_t * pDst, uint32_t blockSize);
__STATIC_INLINE void float_to_int16_packed(float32_t * pSrc, int16_t * pDst, uint32_t blockSize);

/* Variable */
extern codec_config_t boardCodecConfig;

// Data buffers
AT_NONCACHEABLE_SECTION_ALIGN(static int16_t rxBuffer[BUFFER_NUMBER*BUFFER_SIZE], 4) __BSS(SRAM_LOWER);
AT_NONCACHEABLE_SECTION_ALIGN(static int16_t txBuffer[BUFFER_NUMBER*BUFFER_SIZE], 4) __BSS(SRAM_LOWER);
AT_NONCACHEABLE_SECTION_ALIGN(static float32_t fftBuffer[FFT_BUFFER_SIZE], 4) __BSS(SRAM_UPPER);

//static int16_t * dataPointers[BUFFER_NUMBER] = {0};

static arm_rfft_fast_instance_f32 fftInst;

static volatile int16_t * receivedData = NULL;

typedef enum _effect_setting {
	effectSettingNone = 0,
	effectSettingWhisper,
	effectSettingRobot,
	effectSettingNumber
} effect_setting_t;

static volatile effect_setting_t effectSetting = effectSettingNone;

/*
 * @brief   Application entry point.
 */
void main(void) {
	codec_handle_t codecHandle = {0};	// Audio codec handle
	sai_transfer_t xfer;				// SAI transfer settings
	volatile bool done = false;			// Make sure our infinite loop doesn't get optimized
	int16_t * receivedBuffer;			// Used to save receivedData to prevent it changing while being used
	effect_setting_t prevEffect;		// Store previous effect to print new effect on a change
	size_t tx_index = 0;

	prevEffect = effectSettingNone;

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();
    /* Init audio codec */
    BOARD_InitAUDIOPeripheral();
    /* Init SW3 push button */
    BOARD_InitBUTTONsPeripheral();
    /* Init codec */
    CODEC_Init(&codecHandle, &boardCodecConfig);
    CODEC_SetFormat(&codecHandle, BOARD_SAI_AC_RX_MCLK_SOURCE_CLOCK_HZ, BOARD_SAI_AC_rx_format.sampleRate_Hz, BOARD_SAI_AC_rx_format.bitWidth);
    SGTL_SetVolume(&codecHandle, kSGTL_ModuleHP, 0x28);

    // Init RFFT instance
    arm_rfft_fast_init_f32(&fftInst, FFT_SIZE);

    // Start as many receives as we can
	xfer.dataSize = BUFFER_SIZE*sizeof(int16_t);	// Size in bytes

	// Don't wrap rx_index here because we'll only go until we reach buffer size, and deal with it later
    for (size_t rx_index=0; rx_index < BUFFER_NUMBER; rx_index++) {
    	xfer.data = (uint8_t *)&rxBuffer[rx_index*BUFFER_SIZE];
    	if (SAI_TransferReceiveEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_rxHandle, &xfer) == kStatus_SAI_QueueFull) {
    		// If the receive queue is full, we've started as many as we can
    		break;
    	}
    }

	while (!done) {
		// Wait for received data

		while (receivedData == NULL);
		// Do processing and add to transmit buffer
		receivedBuffer = (int16_t *)receivedData;
		receivedData = NULL;

		// Convert from uint16_t to float going from rxBuffer to fftBuffer
		int16_packed_to_float(receivedBuffer, fftBuffer, BUFFER_SIZE);
//		arm_copy_q15(receivedBuffer, &txBuffer[tx_index*BUFFER_SIZE], BUFFER_SIZE);

		// Done with receive buffer, queue another (finishes sequentially)
		xfer.data = (uint8_t *)receivedBuffer;
		xfer.dataSize = BUFFER_SIZE*sizeof(int16_t);	// Size in bytes

		SAI_TransferReceiveEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_rxHandle, &xfer);

		// Apply effect
		if (effectSetting != prevEffect) {
			prevEffect = effectSetting;
			switch (effectSetting) {
				case effectSettingWhisper:
					printf("Whisperization effect\n");
					break;
				case effectSettingRobot:
					printf("Robotization effect\n");
					break;
				case effectSettingNone:
					printf("No effect\n");
				default:
					break;
			}
		}
		switch (effectSetting) {
			case effectSettingWhisper:
				effectWhisper(fftBuffer);
				break;
			case effectSettingRobot:
				effectRobot(fftBuffer);
				break;
			case effectSettingNone:
			default:
				break;
		}

		// Float to int16 from fftBuffer to txBuffer
		float_to_int16_packed(fftBuffer, &txBuffer[tx_index*BUFFER_SIZE], BUFFER_SIZE);

		// Put processed data on send queue
		xfer.data = (uint8_t *)&txBuffer[tx_index*BUFFER_SIZE];
		xfer.dataSize = BUFFER_SIZE*sizeof(int16_t);	// Size in bytes

		/* Loop until queue has space */
		while(SAI_TransferSendEDMA(BOARD_SAI_AC_PERIPHERAL, &BOARD_eDMA_AC_txHandle, &xfer) == kStatus_SAI_QueueFull);
		tx_index += (tx_index + 1) % BUFFER_NUMBER;
	}
}

__STATIC_INLINE inline void effectWhisper(float32_t * fftBuffer) {
	// FFT for both channels in-place
	arm_rfft_fast_f32(&fftInst, fftBuffer, fftBuffer, ARM_FFT);
	arm_rfft_fast_f32(&fftInst, &fftBuffer[FFT_BUFFER_SIZE >> 1U], &fftBuffer[FFT_BUFFER_SIZE >> 1U], ARM_FFT);
	for (size_t i=1; i < FFT_BUFFER_SIZE; i+=2) {
		fftBuffer[i] = (float)rand()/(float)(RAND_MAX/(2*M_PI));
	}
	// IFFT for both channels in-place
	arm_rfft_fast_f32(&fftInst, fftBuffer, fftBuffer, ARM_IFFT);
	arm_rfft_fast_f32(&fftInst, &fftBuffer[FFT_BUFFER_SIZE >> 1U], &fftBuffer[FFT_BUFFER_SIZE >> 1U], ARM_IFFT);
}

__STATIC_INLINE inline void effectRobot(float32_t * fftBuffer) {
	// FFT for both channels in-place
	arm_rfft_fast_f32(&fftInst, fftBuffer, fftBuffer, ARM_FFT);
	arm_rfft_fast_f32(&fftInst, &fftBuffer[FFT_BUFFER_SIZE >> 1U], &fftBuffer[FFT_BUFFER_SIZE >> 1U], ARM_FFT);
	for (size_t i=1; i < FFT_BUFFER_SIZE; i+=2) {
		fftBuffer[i] = 0;
	}
	// IFFT for both channels in-place
	arm_rfft_fast_f32(&fftInst, fftBuffer, fftBuffer, ARM_IFFT);
	arm_rfft_fast_f32(&fftInst, &fftBuffer[FFT_BUFFER_SIZE >> 1U], &fftBuffer[FFT_BUFFER_SIZE >> 1U], ARM_IFFT);
}


void rxCallback(I2S_Type *base, sai_edma_handle_t *handle, status_t status, void *userData)
{
	assert(status != kStatus_SAI_RxError);
	receivedData = (volatile int16_t *)handle->dmaHandle->tcdPool->DADDR;
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void BOARD_SW3_IRQHANDLER(void) {
    GPIO_PortClearInterruptFlags(BOARD_SW3_GPIO, 1U << BOARD_SW3_GPIO_PIN);
	effectSetting = (effectSetting + 1) % effectSettingNumber;
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}


// Convert (int16_t) LRLRLRLR.... to (float32_t) LLLL...RRRR...
__STATIC_INLINE void int16_packed_to_float(int16_t * pSrc, float32_t * pDst, uint32_t blockSize) {
	int16_t *pIn = pSrc++;                         /* Src pointer and add 1 index to original */
	uint32_t blkCnt;                               /* loop counter */


	// Loop unroll from left channel (going up by 2 and loop unroll 4, so 1/8 of iterations)
	blkCnt = blockSize >> 3U;

	// Compute 4 at a time
	while(blkCnt > 0U)
	{
		/* C = (float32_t) A / 32768 */
		/* convert from int16 to float and then store the results in the destination buffer */
		*pDst++ = ((float32_t) *pIn); pIn += 2U;
		*pDst++ = ((float32_t) *pIn); pIn += 2U;
		*pDst++ = ((float32_t) *pIn); pIn += 2U;
		*pDst++ = ((float32_t) *pIn); pIn += 2U;

		// Decrement the loop counter
		blkCnt--;
	}

	// Finish leftover
	blkCnt = blockSize % 0x8u;

	while(blkCnt > 0u)
	{
		/* C = (float32_t) A / 32768 */
		/* convert from int16 to float and then store the results in the destination buffer */
		*pDst++ = ((float32_t) *pIn); pIn += 2U;

		// Decrement block count
		blkCnt--;
	}

	// Loop unroll from right channel (going up by 2 and loop unroll 4, so 1/8 of iterations)
	blkCnt = (blockSize >> 3U);

	// Compute 4 at a time, leftover at end
	while(blkCnt > 0U)
	{
		/* C = (float32_t) A / 32768 */
		/* convert from int16 to float and then store the results in the destination buffer */
		*pDst++ = ((float32_t) *pSrc); pSrc += 2U;
		*pDst++ = ((float32_t) *pSrc); pSrc += 2U;
		*pDst++ = ((float32_t) *pSrc); pSrc += 2U;
		*pDst++ = ((float32_t) *pSrc); pSrc += 2U;

		// Decrement block count
		blkCnt--;
	}

	// Finish leftover
	blkCnt = blockSize % 0x4u;
	while(blkCnt > 0u)
	{
		/* C = (float32_t) A / 32768 */
		/* convert from int16 to float and then store the results in the destination buffer */
		*pDst++ = ((float32_t) *pSrc); pSrc += 2U;

		// Decrement block count
		blkCnt--;
	}
}

// Convert (float32_t) LLLL...RRRR... to (int16_t) LRLRLRLR....
__STATIC_INLINE void float_to_int16_packed(float32_t * pSrc, int16_t * pDst, uint32_t blockSize) {
	int16_t *pOut = pDst++;                        /* Dst pointer and add 1 index to original */
	uint32_t blkCnt;                               /* loop counter */

	// Loop unroll to left channel (going up by 2 and loop unroll 4, so 1/8 of iterations)
	blkCnt = blockSize >> 3u;

	/* Process first set of packed values */
	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	 ** a second loop below computes the remaining 1 to 3 samples. */
	while(blkCnt > 0u)
	{
		/* C = A * 32768 */
		/* convert from float to int16 and then store the results in the destination buffer */
		*pOut = (int16_t)(*pSrc++); pOut += 2U;
		*pOut = (int16_t)(*pSrc++); pOut += 2U;
		*pOut = (int16_t)(*pSrc++); pOut += 2U;
		*pOut = (int16_t)(*pSrc++); pOut += 2U;

		/* Decrement the loop counter */
		blkCnt--;
	}

	// Finish leftover
	blkCnt = blockSize % 0x4u;
	while (blkCnt > 0u) {

		/* C = A * 32768 */
		/* convert from float to int16 and then store the results in the destination buffer */
		*pOut = (int16_t)(*pSrc++); pOut += 2U;

		/* Decrement the loop counter */
		blkCnt--;
	}

	// Loop unroll to right channel (going up by 2 and loop unroll 4, so 1/8 of iterations)
	blkCnt = (blockSize >> 3U);

	/* Process second set of packed values */
	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	 ** a second loop below computes the remaining 1 to 3 samples. */
	while(blkCnt > 0u)
	{
		/* C = A * 32768 */
		/* convert from float to int16 and then store the results in the destination buffer */
		*pDst = (int16_t)(*pSrc++); pDst += 2U;
		*pDst = (int16_t)(*pSrc++); pDst += 2U;
		*pDst = (int16_t)(*pSrc++); pDst += 2U;
		*pDst = (int16_t)(*pSrc++); pDst += 2U;

		/* Decrement the loop counter */
		blkCnt--;
	}

	// Finish leftover
	blkCnt = blockSize % 0x4u;
	while(blkCnt > 0u)
	{

		/* C = A * 32768 */
		/* convert from float to int16 and then store the results in the destination buffer */
		*pDst = (int16_t)(*pSrc++); pDst += 2U;

		/* Decrement the loop counter */
		blkCnt--;
	}
}
