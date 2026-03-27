/**
 ******************************************************************************
 * @file           : vcu_can.h
 * @brief          : FDCAN communication for BMS and VESC
 * @author         : AVT Team
 ******************************************************************************
 * @attention
 *
 * FDCAN1 configuration:
 * - PA11: FDCAN1_RX
 * - PA12: FDCAN1_TX
 * - Bitrate: 500 kbit/s (Classic CAN)
 *
 * Message priorities:
 * - High (1ms): Safety signals
 * - Medium (10ms): Torque commands
 * - Low (100ms): Telemetry
 *
 ******************************************************************************
 */

#ifndef VCU_CAN_H
#define VCU_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h> /* For NULL */
#include <stdint.h>

/* CAN Message IDs -----------------------------------------------------------*/

/**
 * @brief BMS Message IDs (Receive)
 */
#define CAN_ID_BMS_STATUS 0x100U  /**< BMS status: faults, state */
#define CAN_ID_BMS_VOLTAGE 0x101U /**< Pack voltage, cell voltages */
#define CAN_ID_BMS_CURRENT 0x102U /**< Pack current */
#define CAN_ID_BMS_TEMP 0x103U    /**< Cell temperatures */
#define CAN_ID_BMS_SOC 0x104U     /**< State of charge */

/**
 * @brief VESC Message IDs (Send/Receive)
 */
#define CAN_ID_VESC_SET_DUTY 0x000U    /**< Set duty cycle (ID + VESC_ID) */
#define CAN_ID_VESC_SET_CURRENT 0x001U /**< Set current (ID + VESC_ID) */
#define CAN_ID_VESC_SET_RPM 0x003U     /**< Set RPM (ID + VESC_ID) */
#define CAN_ID_VESC_STATUS_1 0x009U    /**< VESC status 1 (ID + VESC_ID) */
#define CAN_ID_VESC_STATUS_2 0x00EU    /**< VESC status 2 (ID + VESC_ID) */
#define CAN_ID_VESC_STATUS_3 0x00FU    /**< VESC status 3 (ID + VESC_ID) */
#define CAN_ID_VESC_STATUS_4 0x010U    /**< VESC status 4 (ID + VESC_ID) */

/**
 * @brief VCU Telemetry IDs (Send)
 */
#define CAN_ID_VCU_STATUS 0x300U /**< VCU state, faults */
#define CAN_ID_VCU_PEDAL 0x301U  /**< Pedal position, brake */

/**
 * @brief Default VESC CAN ID (configurable in VESC Tool)
 */
#define VESC_CAN_ID 0x00U /**< Default VESC ID (0-255) */

/* CAN test mode ------------------------------------------------------------*/
#ifndef VCU_CAN_TEST_MODE
#define VCU_CAN_TEST_MODE 1U
#endif

#ifndef VCU_CAN_TEST_TX
#define VCU_CAN_TEST_TX 1U
#endif

/* Test frame CAN standard ID (0x123). */
#define CAN_ID_VCU_CAN_TEST 0x123U

/* Defines -------------------------------------------------------------------*/

#define CAN_TX_TIMEOUT_MS 10U /**< TX timeout */
#define CAN_RX_FIFO_SIZE 32U  /**< RX message buffer size */

/* Typedefs ------------------------------------------------------------------*/

/**
 * @brief CAN message structure
 */
typedef struct {
  uint32_t id;      /**< Message ID (11-bit or 29-bit) */
  uint8_t data[8];  /**< Data payload */
  uint8_t dlc;      /**< Data length (0-8) */
  bool is_extended; /**< Extended ID flag */
} CAN_Message_t;

/**
 * @brief BMS data received via CAN
 */
typedef struct {
  float pack_voltage;    /**< Battery pack voltage (V) */
  float pack_current;    /**< Battery current (A), positive = discharge */
  float soc_percent;     /**< State of charge (0-100%) */
  float temp_max;        /**< Highest cell temperature (°C) */
  float temp_min;        /**< Lowest cell temperature (°C) */
  uint8_t fault_flags;   /**< BMS fault flags */
  uint32_t last_rx_tick; /**< Last message timestamp */
  bool is_online;        /**< BMS communication active */
} BMS_Data_t;

/**
 * @brief VESC data received via CAN
 */
typedef struct {
  float rpm;             /**< Motor RPM */
  float current_motor;   /**< Motor current (A) */
  float duty_cycle;      /**< Duty cycle (-1.0 to 1.0) */
  float temp_mos;        /**< MOSFET temperature (°C) */
  float temp_motor;      /**< Motor temperature (°C) */
  float voltage_input;   /**< Input voltage (V) */
  uint8_t fault_code;    /**< VESC fault code */
  uint32_t last_rx_tick; /**< Last message timestamp */
  bool is_online;        /**< VESC communication active */
} VESC_Data_t;

/**
 * @brief BMS fault flags
 */
#define BMS_FAULT_OVERVOLT 0x01U
#define BMS_FAULT_UNDERVOLT 0x02U
#define BMS_FAULT_OVERTEMP 0x04U
#define BMS_FAULT_UNDERTEMP 0x08U
#define BMS_FAULT_OVERCURRENT 0x10U
#define BMS_FAULT_SHORT 0x20U
#define BMS_FAULT_COMM_ERROR 0x40U

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief  Initialize CAN module
 * @param  hfdcan: Pointer to FDCAN handle (from CubeMX)
 * @retval bool: true if successful
 */
bool VCU_CAN_Init(void *hfdcan);

/**
 * @brief  Start CAN communication
 * @retval bool: true if successful
 */
bool VCU_CAN_Start(void);

/**
 * @brief  Process received CAN messages (call in main loop)
 * @param  current_tick: Current system tick
 * @retval None
 */
void VCU_CAN_Process(uint32_t current_tick);

/**
 * @brief  Send torque command to VESC
 * @param  current_amps: Motor current command (A)
 * @retval bool: true if sent successfully
 */
bool VCU_CAN_SendTorqueCommand(float current_amps);

/**
 * @brief  Send zero torque (emergency stop)
 * @retval bool: true if sent successfully
 */
bool VCU_CAN_SendZeroTorque(void);

/**
 * @brief  Send VCU status message
 * @param  state: Current VCU state
 * @param  fault_code: Active fault code
 * @retval bool: true if sent successfully
 */
bool VCU_CAN_SendVCUStatus(uint8_t state, uint32_t fault_code);

/**
 * @brief  Get BMS data pointer
 * @retval BMS_Data_t*: Pointer to BMS data structure
 */
BMS_Data_t *VCU_CAN_GetBMSData(void);

/**
 * @brief  Get VESC data pointer
 * @retval VESC_Data_t*: Pointer to VESC data structure
 */
VESC_Data_t *VCU_CAN_GetVESCData(void);

/**
 * @brief  Check if BMS is online (received message recently)
 * @param  current_tick: Current system tick
 * @param  timeout_ms: Timeout in milliseconds
 * @retval bool: true if online
 */
bool VCU_CAN_IsBMSOnline(uint32_t current_tick, uint32_t timeout_ms);

/**
 * @brief  Check if VESC is online
 * @param  current_tick: Current system tick
 * @param  timeout_ms: Timeout in milliseconds
 * @retval bool: true if online
 */
bool VCU_CAN_IsVESCOnline(uint32_t current_tick, uint32_t timeout_ms);

/**
 * @brief  Periodic CAN test TX (and RX acceptance, if enabled).
 * @param  current_tick: Current system tick
 * @retval None
 */
void VCU_CAN_TestProcess(uint32_t current_tick);

/**
 * @brief  FDCAN RX callback (call from HAL_FDCAN_RxFifo0Callback)
 * @param  hfdcan: FDCAN handle
 * @param  RxFifo0ITs: Interrupt flags
 * @retval None
 */
void VCU_CAN_RxCallback(void *hfdcan, uint32_t RxFifo0ITs);

#ifdef __cplusplus
}
#endif

#endif /* VCU_CAN_H */
