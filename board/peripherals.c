/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v4.1
processor: MK22FN512xxx12
package_id: MK22FN512VLH12
mcu_data: ksdk2_0
processor_version: 4.0.1
board: FRDM-K22F
functionalGroups:
- name: BOARD_InitPeripherals
  selectedCore: core0
- name: BOARD_InitBUTTONsPeripheral
  id_prefix: BOARD_
  selectedCore: core0
- name: BOARD_InitLEDsPeripheral
  id_prefix: BOARD_
  selectedCore: core0
- name: BOARD_InitDEBUG_UARTPeripheral
  id_prefix: BOARD_
  selectedCore: core0
- name: BOARD_InitACCELPeripheral
  id_prefix: BOARD_
  selectedCore: core0
- name: BOARD_InitSDHCPeripheral
  id_prefix: BOARD_
  selectedCore: core0
- name: BOARD_InitPOTPeripheral
  id_prefix: BOARD_
  selectedCore: core0
- name: BOARD_InitLSENSEPeripheral
  id_prefix: BOARD_
  selectedCore: core0
- name: BOARD_InitAUDIOPeripheral
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system'
- global_system_definitions: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitBUTTONsPeripheral functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * SW3 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'SW3'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitBUTTONsPeripheral'
- peripheral: 'GPIOB'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTB_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void BOARD_SW3_init(void) {
  /* Make sure, the clock gate for port B is enabled (e. g. in pin_mux.c) */
  /* Enable interrupt PORTB_IRQn request in the NVIC */
  EnableIRQ(PORTB_IRQn);
}

/***********************************************************************************************************************
 * SW2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'SW2'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitBUTTONsPeripheral'
- peripheral: 'GPIOC'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'true'
    - port_interrupt:
      - IRQn: 'PORTC_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void BOARD_SW2_init(void) {
  /* Make sure, the clock gate for port C is enabled (e. g. in pin_mux.c) */
  /* Enable interrupt PORTC_IRQn request in the NVIC */
  EnableIRQ(PORTC_IRQn);
}

/***********************************************************************************************************************
 * BOARD_InitLEDsPeripheral functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * LEDRGB_BLUE initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LEDRGB_BLUE'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitLEDsPeripheral'
- peripheral: 'GPIOD'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTD_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void BOARD_LEDRGB_BLUE_init(void) {
  /* Make sure, the clock gate for port D is enabled (e. g. in pin_mux.c) */
}

/***********************************************************************************************************************
 * LEDRGB_RED_GREEN initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LEDRGB_RED_GREEN'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitLEDsPeripheral'
- peripheral: 'GPIOA'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTA_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void BOARD_LEDRGB_RED_GREEN_init(void) {
  /* Make sure, the clock gate for port A is enabled (e. g. in pin_mux.c) */
}

/***********************************************************************************************************************
 * BOARD_InitDEBUG_UARTPeripheral functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * DEBUG_UART initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'DEBUG_UART'
- type: 'uart'
- mode: 'polling'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'BOARD_InitDEBUG_UARTPeripheral'
- peripheral: 'UART1'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '115200'
      - parityMode: 'kUART_ParityDisabled'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'false'
      - enableRx: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t BOARD_DEBUG_UART_config = {
  .baudRate_Bps = 115200,
  .parityMode = kUART_ParityDisabled,
  .txFifoWatermark = 0,
  .rxFifoWatermark = 1,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = false,
  .enableRx = false
};

void BOARD_DEBUG_UART_init(void) {
  UART_Init(BOARD_DEBUG_UART_PERIPHERAL, &BOARD_DEBUG_UART_config, BOARD_DEBUG_UART_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * BOARD_InitACCELPeripheral functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * ACCEL_I2C initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ACCEL_I2C'
- type: 'i2c'
- mode: 'I2C_Polling'
- type_id: 'i2c_2566d7363e7e9aaedabb432110e372d7'
- functional_group: 'BOARD_InitACCELPeripheral'
- peripheral: 'I2C0'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - i2c_master_config:
      - enableMaster: 'true'
      - enableStopHold: 'false'
      - baudRate_Bps: '100000'
      - glitchFilterWidth: '0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const i2c_master_config_t BOARD_ACCEL_I2C_config = {
  .enableMaster = true,
  .enableStopHold = false,
  .baudRate_Bps = 100000,
  .glitchFilterWidth = 0
};

void BOARD_ACCEL_I2C_init(void) {
  /* Initialization function */
  I2C_MasterInit(BOARD_ACCEL_I2C_PERIPHERAL, &BOARD_ACCEL_I2C_config, BOARD_ACCEL_I2C_CLK_FREQ);
}

/***********************************************************************************************************************
 * ACCEL_INT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ACCEL_INT'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitACCELPeripheral'
- peripheral: 'GPIOD'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTD_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void BOARD_ACCEL_INT_init(void) {
  /* Make sure, the clock gate for port D is enabled (e. g. in pin_mux.c) */
}

/***********************************************************************************************************************
 * BOARD_InitSDHCPeripheral functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * SDHC_DETECT initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'SDHC_DETECT'
- type: 'gpio'
- mode: 'GPIO'
- type_id: 'gpio_f970a92e447fa4793838db25a2947ed7'
- functional_group: 'BOARD_InitSDHCPeripheral'
- peripheral: 'GPIOB'
- config_sets:
  - fsl_gpio:
    - enable_irq: 'false'
    - port_interrupt:
      - IRQn: 'PORTB_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

void BOARD_SDHC_DETECT_init(void) {
  /* Make sure, the clock gate for port B is enabled (e. g. in pin_mux.c) */
}

/***********************************************************************************************************************
 * SDHC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'SDHC'
- type: 'dspi'
- mode: 'DSPI_Polling'
- type_id: 'dspi_305e5b03c593d065f61ded8061d15797'
- functional_group: 'BOARD_InitSDHCPeripheral'
- peripheral: 'SPI0'
- config_sets:
  - fsl_dspi:
    - dspi_mode: 'kDSPI_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - dspi_master_config:
      - whichCtar: 'kDSPI_Ctar0'
      - ctarConfig:
        - baudRate: '8000000'
        - bitsPerFrame: '8'
        - cpol: 'kDSPI_ClockPolarityActiveLow'
        - cpha: 'kDSPI_ClockPhaseSecondEdge'
        - direction: 'kDSPI_MsbFirst'
        - pcsToSckDelayInNanoSec: '64'
        - lastSckToPcsDelayInNanoSec: '0'
        - betweenTransferDelayInNanoSec: '0'
      - whichPcs: 'PCS0_SS'
      - pcsActiveHighOrLow: 'kDSPI_PcsActiveLow'
      - enableContinuousSCK: 'false'
      - enableRxFifoOverWrite: 'false'
      - enableModifiedTimingFormat: 'false'
      - samplePoint: 'kDSPI_SckToSin0Clock'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const dspi_master_config_t BOARD_SDHC_config = {
  .whichCtar = kDSPI_Ctar0,
  .ctarConfig = {
    .baudRate = 8000000,
    .bitsPerFrame = 8,
    .cpol = kDSPI_ClockPolarityActiveLow,
    .cpha = kDSPI_ClockPhaseSecondEdge,
    .direction = kDSPI_MsbFirst,
    .pcsToSckDelayInNanoSec = 64,
    .lastSckToPcsDelayInNanoSec = 0,
    .betweenTransferDelayInNanoSec = 0
  },
  .whichPcs = kDSPI_Pcs0,
  .pcsActiveHighOrLow = kDSPI_PcsActiveLow,
  .enableContinuousSCK = false,
  .enableRxFifoOverWrite = false,
  .enableModifiedTimingFormat = false,
  .samplePoint = kDSPI_SckToSin0Clock
};

void BOARD_SDHC_init(void) {
  /* Initialization function */
  DSPI_MasterInit(BOARD_SDHC_PERIPHERAL, &BOARD_SDHC_config, BOARD_SDHC_CLK_FREQ);
}

/***********************************************************************************************************************
 * BOARD_InitPOTPeripheral functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * POTENTIOMETER initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'POTENTIOMETER'
- type: 'adc16'
- mode: 'ADC'
- type_id: 'adc16_7d827be2dc433dc756d94a7ce88cbcc5'
- functional_group: 'BOARD_InitPOTPeripheral'
- peripheral: 'ADC0'
- config_sets:
  - fsl_adc16:
    - adc16_config:
      - referenceVoltageSource: 'kADC16_ReferenceVoltageSourceVref'
      - clockSource: 'kADC16_ClockSourceAsynchronousClock'
      - enableAsynchronousClock: 'true'
      - clockDivider: 'kADC16_ClockDivider8'
      - resolution: 'kADC16_ResolutionSE12Bit'
      - longSampleMode: 'kADC16_LongSampleDisabled'
      - enableHighSpeed: 'false'
      - enableLowPower: 'false'
      - enableContinuousConversion: 'false'
    - adc16_channel_mux_mode: 'kADC16_ChannelMuxA'
    - adc16_hardware_compare_config:
      - hardwareCompareModeEnable: 'false'
    - doAutoCalibration: 'true'
    - trigger: 'false'
    - hardwareAverageConfiguration: 'kADC16_HardwareAverageDisabled'
    - enable_irq: 'false'
    - adc_interrupt:
      - IRQn: 'ADC0_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - adc16_channels_config:
      - 0:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.12'
        - enableInterruptOnConversionCompleted: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
adc16_channel_config_t BOARD_POTENTIOMETER_channelsConfig[1] = {
  {
    .channelNumber = 12U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false
  }
};
const adc16_config_t BOARD_POTENTIOMETER_config = {
  .referenceVoltageSource = kADC16_ReferenceVoltageSourceVref,
  .clockSource = kADC16_ClockSourceAsynchronousClock,
  .enableAsynchronousClock = true,
  .clockDivider = kADC16_ClockDivider8,
  .resolution = kADC16_ResolutionSE12Bit,
  .longSampleMode = kADC16_LongSampleDisabled,
  .enableHighSpeed = false,
  .enableLowPower = false,
  .enableContinuousConversion = false
};
const adc16_channel_mux_mode_t BOARD_POTENTIOMETER_muxMode = kADC16_ChannelMuxA;
const adc16_hardware_average_mode_t BOARD_POTENTIOMETER_hardwareAverageMode = kADC16_HardwareAverageDisabled;

void BOARD_POTENTIOMETER_init(void) {
  /* Initialize ADC16 converter */
  ADC16_Init(BOARD_POTENTIOMETER_PERIPHERAL, &BOARD_POTENTIOMETER_config);
  /* Make sure, that software trigger is used */
  ADC16_EnableHardwareTrigger(BOARD_POTENTIOMETER_PERIPHERAL, false);
  /* Configure hardware average mode */
  ADC16_SetHardwareAverage(BOARD_POTENTIOMETER_PERIPHERAL, BOARD_POTENTIOMETER_hardwareAverageMode);
  /* Configure channel multiplexing mode */
  ADC16_SetChannelMuxMode(BOARD_POTENTIOMETER_PERIPHERAL, BOARD_POTENTIOMETER_muxMode);
  /* Perform auto calibration */
  ADC16_DoAutoCalibration(BOARD_POTENTIOMETER_PERIPHERAL);
}

/***********************************************************************************************************************
 * BOARD_InitLSENSEPeripheral functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * LSENSE initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'LSENSE'
- type: 'adc16'
- mode: 'ADC'
- type_id: 'adc16_7d827be2dc433dc756d94a7ce88cbcc5'
- functional_group: 'BOARD_InitLSENSEPeripheral'
- peripheral: 'ADC0'
- config_sets:
  - fsl_adc16:
    - adc16_config:
      - referenceVoltageSource: 'kADC16_ReferenceVoltageSourceVref'
      - clockSource: 'kADC16_ClockSourceAsynchronousClock'
      - enableAsynchronousClock: 'true'
      - clockDivider: 'kADC16_ClockDivider8'
      - resolution: 'kADC16_ResolutionSE12Bit'
      - longSampleMode: 'kADC16_LongSampleDisabled'
      - enableHighSpeed: 'false'
      - enableLowPower: 'false'
      - enableContinuousConversion: 'false'
    - adc16_channel_mux_mode: 'kADC16_ChannelMuxA'
    - adc16_hardware_compare_config:
      - hardwareCompareModeEnable: 'false'
    - doAutoCalibration: 'true'
    - trigger: 'false'
    - hardwareAverageConfiguration: 'kADC16_HardwareAverageDisabled'
    - enable_irq: 'false'
    - adc_interrupt:
      - IRQn: 'ADC0_IRQn'
      - enable_priority: 'false'
      - enable_custom_name: 'false'
    - adc16_channels_config:
      - 0:
        - enableDifferentialConversion: 'false'
        - channelNumber: 'SE.3'
        - enableInterruptOnConversionCompleted: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
adc16_channel_config_t BOARD_LSENSE_channelsConfig[1] = {
  {
    .channelNumber = 3U,
    .enableDifferentialConversion = false,
    .enableInterruptOnConversionCompleted = false
  }
};
const adc16_config_t BOARD_LSENSE_config = {
  .referenceVoltageSource = kADC16_ReferenceVoltageSourceVref,
  .clockSource = kADC16_ClockSourceAsynchronousClock,
  .enableAsynchronousClock = true,
  .clockDivider = kADC16_ClockDivider8,
  .resolution = kADC16_ResolutionSE12Bit,
  .longSampleMode = kADC16_LongSampleDisabled,
  .enableHighSpeed = false,
  .enableLowPower = false,
  .enableContinuousConversion = false
};
const adc16_channel_mux_mode_t BOARD_LSENSE_muxMode = kADC16_ChannelMuxA;
const adc16_hardware_average_mode_t BOARD_LSENSE_hardwareAverageMode = kADC16_HardwareAverageDisabled;

void BOARD_LSENSE_init(void) {
  /* Initialize ADC16 converter */
  ADC16_Init(BOARD_LSENSE_PERIPHERAL, &BOARD_LSENSE_config);
  /* Make sure, that software trigger is used */
  ADC16_EnableHardwareTrigger(BOARD_LSENSE_PERIPHERAL, false);
  /* Configure hardware average mode */
  ADC16_SetHardwareAverage(BOARD_LSENSE_PERIPHERAL, BOARD_LSENSE_hardwareAverageMode);
  /* Configure channel multiplexing mode */
  ADC16_SetChannelMuxMode(BOARD_LSENSE_PERIPHERAL, BOARD_LSENSE_muxMode);
  /* Perform auto calibration */
  ADC16_DoAutoCalibration(BOARD_LSENSE_PERIPHERAL);
}

/***********************************************************************************************************************
 * BOARD_InitAUDIOPeripheral functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * eDMA initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'eDMA'
- type: 'edma'
- mode: 'basic'
- type_id: 'edma_a23fca76a894e1bcdf9d01a687505ff9'
- functional_group: 'BOARD_InitAUDIOPeripheral'
- peripheral: 'DMA'
- config_sets:
  - fsl_edma:
    - common_settings:
      - enableContinuousLinkMode: 'false'
      - enableHaltOnError: 'true'
      - enableRoundRobinArbitration: 'false'
      - enableDebugMode: 'false'
    - edma_channels: []
    - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const edma_config_t eDMA_config = {
  .enableContinuousLinkMode = false,
  .enableHaltOnError = true,
  .enableRoundRobinArbitration = false,
  .enableDebugMode = false
};

void eDMA_init(void) {
}

/***********************************************************************************************************************
 * I2C_AC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'I2C_AC'
- type: 'i2c'
- mode: 'I2C_Polling'
- type_id: 'i2c_2566d7363e7e9aaedabb432110e372d7'
- functional_group: 'BOARD_InitAUDIOPeripheral'
- peripheral: 'I2C0'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - i2c_master_config:
      - enableMaster: 'true'
      - enableStopHold: 'true'
      - baudRate_Bps: '100000'
      - glitchFilterWidth: '0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const i2c_master_config_t I2C_AC_config = {
  .enableMaster = true,
  .enableStopHold = true,
  .baudRate_Bps = 100000,
  .glitchFilterWidth = 0
};

void I2C_AC_init(void) {
  /* Initialization function */
  I2C_MasterInit(I2C_AC_PERIPHERAL, &I2C_AC_config, I2C_AC_CLK_FREQ);
}

/***********************************************************************************************************************
 * SAI_AC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'SAI_AC'
- type: 'sai'
- mode: 'edma'
- type_id: 'sai_e171ee1d4e17db4b5b234f946b59a148'
- functional_group: 'BOARD_InitAUDIOPeripheral'
- peripheral: 'I2S0'
- config_sets:
  - fsl_sai:
    - mclk_config:
      - masterClockSource: 'kSAI_MclkSourceSysclk'
      - masterClockSourceFreq: 'BOARD_BootClockHSRUN'
      - masterClockFrequency: '11.2896 MHz'
    - usage: 'record_playback'
    - whole:
      - tx_group:
        - sai_config:
          - protocol: 'kSAI_BusI2S'
          - syncMode: 'kSAI_ModeAsync'
          - bitClockSource: 'kSAI_BclkSourceMclkDiv'
        - transfer_format:
          - sampleRate_Hz: 'kSAI_SampleRate44100Hz'
          - bitWidth: 'kSAI_WordWidth16bits'
          - stereo: 'kSAI_MonoLeft'
          - isFrameSyncCompact: 'false'
          - watermark: '1'
          - channelMask: 'kSAI_Channel0Mask'
        - edma_group:
          - enable_edma_channel: 'true'
          - edma_channel:
            - eDMAn: '1'
            - eDMA_source: 'kDmaRequestMux0I2S0Tx'
            - enable_custom_name: 'true'
            - handle_custom_name: 'SAI_AC_txHandle'
          - sai_edma_handle:
            - enable_custom_name: 'true'
            - handle_custom_name: 'SAI_eDMA_txHandle'
            - init_callback: 'false'
            - callback_fcn: ''
            - user_data: ''
      - rx_group:
        - sai_config:
          - protocol: 'kSAI_BusI2S'
          - syncMode: 'kSAI_ModeSync'
          - bitClockSource: 'kSAI_BclkSourceMclkOption1'
          - bitClockSourceFreq: 'BOARD_BootClockRUN'
        - transfer_format:
          - sampleRate_Hz: 'kSAI_SampleRate44100Hz'
          - bitWidth: 'kSAI_WordWidth16bits'
          - stereo: 'kSAI_MonoLeft'
          - isFrameSyncCompact: 'false'
          - watermark: '1'
          - channelMask: 'kSAI_Channel0Mask'
        - edma_group:
          - enable_edma_channel: 'true'
          - edma_channel:
            - eDMAn: '0'
            - eDMA_source: 'kDmaRequestMux0I2S0Rx'
            - enable_custom_name: 'true'
            - handle_custom_name: 'SAI_AC_rxHandle'
          - sai_edma_handle:
            - enable_custom_name: 'true'
            - handle_custom_name: 'SAI_eDMA_rxHandle'
            - init_callback: 'true'
            - callback_fcn: 'SAI_eDMA_rxCallback'
            - user_data: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
/* SAI_AC Tx configuration */
const sai_config_t SAI_AC_tx_config = {
  .protocol = kSAI_BusI2S,
  .syncMode = kSAI_ModeAsync,
  .mclkOutputEnable = true,
  /* MCLK clock source */
  .mclkSource = kSAI_MclkSourceSysclk,
  .bclkSource = kSAI_BclkSourceMclkDiv, //kSAI_BclkSourceMclkOption1
  .masterSlave = kSAI_Master
};
/* SAI_AC Tx  transfer format */
sai_transfer_format_t SAI_AC_tx_format = {
  .sampleRate_Hz = kSAI_SampleRate44100Hz,
  .bitWidth = kSAI_WordWidth16bits,
  .stereo = kSAI_MonoLeft,
  .masterClockHz = 11289600UL,
  .watermark = 1U,
  .channel = 0U,
  .protocol = kSAI_BusI2S,
  .isFrameSyncCompact = false
};
/* SAI_AC Rx configuration */
sai_config_t SAI_AC_rx_config = {
  .protocol = kSAI_BusI2S,
  .syncMode = kSAI_ModeSync,
  .mclkOutputEnable = true,
  /* MCLK clock source */
  .mclkSource = kSAI_MclkSourceSysclk,
  /* Rx .bitClockSource will be initialized later due to the synchronization mode selected. */
  /* Also due to the synchronization mode is selected, masterSlave item below has no meaning, for valid settings see the other Tx/Rx masterSlave settings. */
  .masterSlave = kSAI_Master
};
/* SAI_AC Rx  transfer format */
sai_transfer_format_t SAI_AC_rx_format = {
  .sampleRate_Hz = kSAI_SampleRate44100Hz,
  .bitWidth = kSAI_WordWidth16bits,
  .stereo = kSAI_MonoLeft,
  .masterClockHz = 11289600UL,
  .watermark = 1U,
  .channel = 0U,
  .protocol = kSAI_BusI2S,
  .isFrameSyncCompact = false
};
edma_handle_t SAI_AC_txHandle;
edma_handle_t SAI_AC_rxHandle;
sai_edma_handle_t SAI_eDMA_txHandle;
sai_edma_handle_t SAI_eDMA_rxHandle;

void SAI_AC_init(void) {
  /* Set the source kDmaRequestMux0I2S0Tx request in the DMAMUX */
  DMAMUX_SetSource(SAI_AC_TX_DMAMUX_BASEADDR, SAI_AC_TX_DMA_CHANNEL, SAI_AC_TX_DMA_REQUEST);
  /* Enable the 1 channel in the DMAMUX */
  DMAMUX_EnableChannel(SAI_AC_TX_DMAMUX_BASEADDR, SAI_AC_TX_DMA_CHANNEL);
  /* Set the source kDmaRequestMux0I2S0Rx request in the DMAMUX */
  DMAMUX_SetSource(SAI_AC_RX_DMAMUX_BASEADDR, SAI_AC_RX_DMA_CHANNEL, SAI_AC_RX_DMA_REQUEST);
  /* Enable the 0 channel in the DMAMUX */
  DMAMUX_EnableChannel(SAI_AC_RX_DMAMUX_BASEADDR, SAI_AC_RX_DMA_CHANNEL);
  /* Create the eDMA SAI_AC_txHandle handle */
  EDMA_CreateHandle(&SAI_AC_txHandle, SAI_AC_TX_DMA_BASEADDR, SAI_AC_TX_DMA_CHANNEL);
  /* Create the eDMA SAI_AC_rxHandle handle */
  EDMA_CreateHandle(&SAI_AC_rxHandle, SAI_AC_RX_DMA_BASEADDR, SAI_AC_RX_DMA_CHANNEL);
  /* Configure SAI_AC_rx_config.bclkSource in case of synchronous mode with second (Tx/Rx) part is selected. */
  SAI_AC_rx_config.bclkSource = SAI_AC_tx_config.bclkSource;
  /* Initialize SAI Tx sub-module functionality */
  SAI_TxInit(SAI_AC_PERIPHERAL, &SAI_AC_tx_config);
  /* Initialize SAI Rx sub-module functionality */
  SAI_RxInit(SAI_AC_PERIPHERAL, &SAI_AC_rx_config);
  /* Create the SAI Tx eDMA handle */
  SAI_TransferTxCreateHandleEDMA(SAI_AC_PERIPHERAL, &SAI_eDMA_txHandle, NULL, NULL, &SAI_AC_txHandle);
  /* Create the SAI Rx eDMA handle */
  SAI_TransferRxCreateHandleEDMA(SAI_AC_PERIPHERAL, &SAI_eDMA_rxHandle, SAI_eDMA_rxCallback, NULL, &SAI_AC_rxHandle);
  /* Initialize SAI Tx transfer format */
  SAI_TransferTxSetFormatEDMA(SAI_AC_PERIPHERAL, &SAI_eDMA_txHandle, &SAI_AC_tx_format, SAI_AC_TX_MCLK_SOURCE_CLOCK_HZ, SAI_AC_TX_BCLK_SOURCE_CLOCK_HZ);
  /* Initialize SAI Rx transfer format */
  SAI_TransferRxSetFormatEDMA(SAI_AC_PERIPHERAL, &SAI_eDMA_rxHandle, &SAI_AC_rx_format, SAI_AC_RX_MCLK_SOURCE_CLOCK_HZ, SAI_AC_RX_BCLK_SOURCE_CLOCK_HZ);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
}

void BOARD_InitBUTTONsPeripheral(void)
{
  /* Initialize components */
  BOARD_SW3_init();
  BOARD_SW2_init();
}

void BOARD_InitLEDsPeripheral(void)
{
  /* Initialize components */
  BOARD_LEDRGB_BLUE_init();
  BOARD_LEDRGB_RED_GREEN_init();
}

void BOARD_InitDEBUG_UARTPeripheral(void)
{
  /* Initialize components */
  BOARD_DEBUG_UART_init();
}

void BOARD_InitACCELPeripheral(void)
{
  /* Initialize components */
  BOARD_ACCEL_I2C_init();
  BOARD_ACCEL_INT_init();
}

void BOARD_InitSDHCPeripheral(void)
{
  /* Initialize components */
  BOARD_SDHC_DETECT_init();
  BOARD_SDHC_init();
}

void BOARD_InitPOTPeripheral(void)
{
  /* Initialize components */
  BOARD_POTENTIOMETER_init();
}

void BOARD_InitLSENSEPeripheral(void)
{
  /* Initialize components */
  BOARD_LSENSE_init();
}

void BOARD_InitAUDIOPeripheral(void)
{
  /* Global initialization */
  DMAMUX_Init(EDMA_DMAMUX_BASEADDR);
  EDMA_Init(EDMA_DMA_BASEADDR, &eDMA_config);

  /* Initialize components */
  eDMA_init();
  I2C_AC_init();
  SAI_AC_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
}
