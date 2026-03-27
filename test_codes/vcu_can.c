/**
 ******************************************************************************
 * @file           : vcu_can.c
 * @brief          : FDCAN communication implementation
 * @author         : AVT Team
 ******************************************************************************
 * @attention
 *
 * Handles CAN communication with:
 * - BMS: Receive battery status, voltage, current, SOC
 * - VESC: Send torque commands, receive motor status
 *
 * Pin configuration:
 * - PA11: FDCAN1_RX
 * - PA12: FDCAN1_TX
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "vcu_can.h"
#include "stm32g4xx_hal.h"

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static FDCAN_HandleTypeDef *hfdcan_ptr = NULL;
static BMS_Data_t bms_data = {0};
static VESC_Data_t vesc_data = {0};
static FDCAN_TxHeaderTypeDef tx_header;
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];

/* Private function prototypes -----------------------------------------------*/
static void parse_bms_status(uint8_t *data, uint8_t dlc);
static void parse_bms_voltage(uint8_t *data, uint8_t dlc);
static void parse_bms_current(uint8_t *data, uint8_t dlc);
static void parse_bms_soc(uint8_t *data, uint8_t dlc);
static void parse_vesc_status(uint32_t id, uint8_t *data, uint8_t dlc);
static bool send_can_message(uint32_t id, uint8_t *data, uint8_t dlc);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Initialize CAN module
 */
bool VCU_CAN_Init(void *hfdcan) {
  if (hfdcan == NULL) {
    return false;
  }

  hfdcan_ptr = (FDCAN_HandleTypeDef *)hfdcan;

  /* Initialize TX header for standard CAN */
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  /* Initialize data structures */
  bms_data.is_online = false;
  bms_data.last_rx_tick = 0;
  vesc_data.is_online = false;
  vesc_data.last_rx_tick = 0;

  return true;
}

/**
 * @brief  Start CAN communication with filters
 */

bool VCU_CAN_Start(void) {
  if (hfdcan_ptr == NULL) {
    return false;
  }

  FDCAN_FilterTypeDef filter_config;

  /* Filter 0: Accept all BMS messages (0x100-0x10F) */
  filter_config.IdType = FDCAN_STANDARD_ID;
  filter_config.FilterIndex = 0;
  filter_config.FilterType = FDCAN_FILTER_RANGE;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = 0x100;
  filter_config.FilterID2 = 0x10F;

  if (HAL_FDCAN_ConfigFilter(hfdcan_ptr, &filter_config) != HAL_OK) {
    return false;
  }

  /* Filter 1: Accept VESC status messages */
  filter_config.FilterIndex = 1;
  filter_config.FilterType = FDCAN_FILTER_RANGE;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = 0x900 + VESC_CAN_ID;
  filter_config.FilterID2 = 0x910 + VESC_CAN_ID;

  if (HAL_FDCAN_ConfigFilter(hfdcan_ptr, &filter_config) != HAL_OK) {
    return false;
  }

  /* Filter 2: Accept test frame 0x123 */
  filter_config.FilterIndex = 2;
  filter_config.FilterType = FDCAN_FILTER_MASK;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = CAN_ID_VCU_CAN_TEST;
  filter_config.FilterID2 = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(hfdcan_ptr, &filter_config) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan_ptr,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_Start(hfdcan_ptr) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_ActivateNotification(hfdcan_ptr,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0) != HAL_OK) {
    return false;
  }

  return true;
}

/**
 * @brief  Process received CAN messages
 */
void VCU_CAN_Process(uint32_t current_tick) {
  /* Check BMS online status */
  if (bms_data.last_rx_tick > 0) {
    uint32_t elapsed = current_tick - bms_data.last_rx_tick;
    bms_data.is_online = (elapsed < 500); /* 500ms timeout */
  }

  /* Check VESC online status */
  if (vesc_data.last_rx_tick > 0) {
    uint32_t elapsed = current_tick - vesc_data.last_rx_tick;
    vesc_data.is_online = (elapsed < 200); /* 200ms timeout */
  }
}

/**
 * @brief  Send torque command to VESC (current control)
 */
bool VCU_CAN_SendTorqueCommand(float current_amps) {
  uint8_t data[4];
  int32_t current_ma = (int32_t)(current_amps * 1000.0f); /* Convert to mA */

  /* Big-endian format for VESC */
  data[0] = (uint8_t)((current_ma >> 24) & 0xFF);
  data[1] = (uint8_t)((current_ma >> 16) & 0xFF);
  data[2] = (uint8_t)((current_ma >> 8) & 0xFF);
  data[3] = (uint8_t)(current_ma & 0xFF);

  /* VESC CAN ID for set current: (0x001 << 8) | VESC_ID */
  uint32_t id = (CAN_ID_VESC_SET_CURRENT << 8) | VESC_CAN_ID;

  return send_can_message(id, data, 4);
}

/**
 * @brief  Send zero torque command
 */
bool VCU_CAN_SendZeroTorque(void) { return VCU_CAN_SendTorqueCommand(0.0f); }

/**
 * @brief  Send VCU status message
 */
bool VCU_CAN_SendVCUStatus(uint8_t state, uint32_t fault_code) {
  uint8_t data[8];

  data[0] = state;
  data[1] = (uint8_t)((fault_code >> 24) & 0xFF);
  data[2] = (uint8_t)((fault_code >> 16) & 0xFF);
  data[3] = (uint8_t)((fault_code >> 8) & 0xFF);
  data[4] = (uint8_t)(fault_code & 0xFF);
  data[5] = 0; /* Reserved */
  data[6] = 0; /* Reserved */
  data[7] = 0; /* Reserved */

  return send_can_message(CAN_ID_VCU_STATUS, data, 8);
}

/**
 * @brief  Get BMS data pointer
 */
BMS_Data_t *VCU_CAN_GetBMSData(void) { return &bms_data; }

/**
 * @brief  Get VESC data pointer
 */
VESC_Data_t *VCU_CAN_GetVESCData(void) { return &vesc_data; }

/**
 * @brief  Check if BMS is online
 */
bool VCU_CAN_IsBMSOnline(uint32_t current_tick, uint32_t timeout_ms) {
  if (bms_data.last_rx_tick == 0) {
    return false;
  }
  return ((current_tick - bms_data.last_rx_tick) < timeout_ms);
}

/**
 * @brief  Check if VESC is online
 */
bool VCU_CAN_IsVESCOnline(uint32_t current_tick, uint32_t timeout_ms) {
  if (vesc_data.last_rx_tick == 0) {
    return false;
  }
  return ((current_tick - vesc_data.last_rx_tick) < timeout_ms);
}

/**
 * @brief  FDCAN RX callback handler
 */
void VCU_CAN_RxCallback(void *hfdcan, uint32_t RxFifo0ITs) {
  FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)hfdcan;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) {
    return;
  }

  /* Get message from FIFO */
  if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &rx_header, rx_data) !=
      HAL_OK) {
    return;
  }

  uint32_t id = rx_header.Identifier;
  uint8_t dlc = rx_header.DataLength >> 16; /* Convert DLC code to bytes */

  /* Route message based on ID */
  switch (id) {
  case CAN_ID_BMS_STATUS:
    parse_bms_status(rx_data, dlc);
    bms_data.last_rx_tick = HAL_GetTick();
    break;

  case CAN_ID_BMS_VOLTAGE:
    parse_bms_voltage(rx_data, dlc);
    bms_data.last_rx_tick = HAL_GetTick();
    break;

  case CAN_ID_BMS_CURRENT:
    parse_bms_current(rx_data, dlc);
    bms_data.last_rx_tick = HAL_GetTick();
    break;

  case CAN_ID_BMS_SOC:
    parse_bms_soc(rx_data, dlc);
    bms_data.last_rx_tick = HAL_GetTick();
    break;

  default:
    /* Check for VESC status messages */
    if ((id & 0xFF00) == 0x0900) {
      parse_vesc_status(id, rx_data, dlc);
      vesc_data.last_rx_tick = HAL_GetTick();
    }
    break;
  }

  /* Re-activate notification */
  HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Send CAN message
 */
static bool send_can_message(uint32_t id, uint8_t *data, uint8_t dlc) {
  if (hfdcan_ptr == NULL) {
    return false;
  }

  tx_header.Identifier = id;
  tx_header.DataLength = (uint32_t)dlc << 16; /* Convert to FDCAN DLC format */

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_ptr, &tx_header, data) != HAL_OK) {
    return false;
  }

  return true;
}

/**
 * @brief  Parse BMS status message
 */
static void parse_bms_status(uint8_t *data, uint8_t dlc) {
  (void)dlc;
  bms_data.fault_flags = data[0];
}

/**
 * @brief  Parse BMS voltage message
 */
static void parse_bms_voltage(uint8_t *data, uint8_t dlc) {
  (void)dlc;
  /* Assuming voltage is sent as uint16 in 0.01V units */
  uint16_t voltage_raw = ((uint16_t)data[0] << 8) | data[1];
  bms_data.pack_voltage = (float)voltage_raw * 0.01f;
}

/**
 * @brief  Parse BMS current message
 */
static void parse_bms_current(uint8_t *data, uint8_t dlc) {
  (void)dlc;
  /* Assuming current is sent as int16 in 0.01A units */
  int16_t current_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  bms_data.pack_current = (float)current_raw * 0.01f;
}

/**
 * @brief  Parse BMS SOC message
 */
static void parse_bms_soc(uint8_t *data, uint8_t dlc) {
  (void)dlc;
  /* Assuming SOC is sent as uint8 in 1% units */
  bms_data.soc_percent = (float)data[0];
}

/**
 * @brief  Parse VESC status message
 */
static void parse_vesc_status(uint32_t id, uint8_t *data, uint8_t dlc) {
  (void)dlc;
  uint8_t status_type = (id >> 8) & 0xFF;

  switch (status_type) {
  case 0x09: /* Status 1: RPM, Current, Duty */
  {
    int32_t rpm_raw = ((int32_t)data[0] << 24) | ((int32_t)data[1] << 16) |
                      ((int32_t)data[2] << 8) | data[3];
    vesc_data.rpm = (float)rpm_raw;

    int16_t current_raw = ((int16_t)data[4] << 8) | data[5];
    vesc_data.current_motor = (float)current_raw * 0.1f;

    int16_t duty_raw = ((int16_t)data[6] << 8) | data[7];
    vesc_data.duty_cycle = (float)duty_raw * 0.001f;
  } break;

  case 0x0E: /* Status 2: Ah, Ah charged */
    /* Not used currently */
    break;

  case 0x0F: /* Status 3: Wh, Wh charged */
    /* Not used currently */
    break;

  case 0x10: /* Status 4: Temp, Voltage */
  {
    int16_t temp_mos_raw = ((int16_t)data[0] << 8) | data[1];
    vesc_data.temp_mos = (float)temp_mos_raw * 0.1f;

    int16_t temp_motor_raw = ((int16_t)data[2] << 8) | data[3];
    vesc_data.temp_motor = (float)temp_motor_raw * 0.1f;

    int16_t vin_raw = ((int16_t)data[4] << 8) | data[5];
    vesc_data.voltage_input = (float)vin_raw * 0.1f;
  } break;

  default:
    break;
  }
}
void VCU_CAN_TestProcess(uint32_t current_tick)
{
#if (VCU_CAN_TEST_MODE != 0U) && (VCU_CAN_TEST_TX != 0U)

  static uint32_t last_tick = 0;
  static uint32_t counter = 0;

  if ((last_tick == 0U) || ((current_tick - last_tick) >= 100U))
  {
    uint8_t data[8];

    uint32_t c = counter++;

    data[0] = (uint8_t)((c >> 24) & 0xFF);
    data[1] = (uint8_t)((c >> 16) & 0xFF);
    data[2] = (uint8_t)((c >> 8) & 0xFF);
    data[3] = (uint8_t)(c & 0xFF);

    data[4] = 0xAA;
    data[5] = 0x55;
    data[6] = 0x12;
    data[7] = 0x23;

    send_can_message(CAN_ID_VCU_CAN_TEST, data, 8);

    last_tick = current_tick;
  }

#endif
}
