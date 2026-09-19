#ifndef XBEE_ECU_H
#define XBEE_ECU_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- Protocol ---------------- */

#define XBEE_SOF1                 0xAAU
#define XBEE_SOF2                 0x55U

#define XBEE_CMD_MONITOR          0x01U
#define XBEE_CMD_WRITE_BIN        0x02U

#define XBEE_RSP_MONITOR          0x81U
#define XBEE_RSP_BIN_ACK          0x82U

#define XBEE_BIN_OK               0x00U
#define XBEE_BIN_ENGINE_RUNNING   0x01U
#define XBEE_BIN_BAD_LENGTH       0x02U
#define XBEE_BIN_CRC_ERROR        0x03U
#define XBEE_BIN_EEPROM_ERROR     0x04U
#define XBEE_BIN_RX_TIMEOUT       0x05U
#define XBEE_BIN_BUSY             0x06U

/*
 * The Python program currently generates:
 * 16 bytes RPM axis
 * 12 bytes TPS axis
 * 96 bytes AFR map
 * 96 bytes ignition map
 *  2 bytes BIN CRC
 * = 222 bytes
 */
#define XBEE_BIN_SIZE             222U
#define XBEE_RX_BUFFER_SIZE       256U

/*
 * Safety threshold.
 *
 * For maximum safety this example permits BIN update only when
 * RPM == 0 exactly.
 */
#define XBEE_ENGINE_STOP_RPM      0U

/* ---------------- User callbacks ---------------- */

/*
 * Return the CURRENT measured engine RPM.
 *
 * This must be implemented by the ECU application.
 * Do not return a cached value that can remain at zero while
 * the engine is actually running.
 */
typedef int (*XBee_GetRpmCallback_t)(void);

/*
 * Application-specific EEPROM functions.
 *
 * Recommended implementation:
 *   1. Write to an inactive/staging EEPROM area.
 *   2. Read back and verify.
 *   3. Only after verification, atomically switch the active map.
 *
 * If your existing EEPROM code directly overwrites the active map,
 * replace these callbacks with your existing functions carefully.
 */
typedef HAL_StatusTypeDef (*XBee_WriteBinCallback_t)(
    const uint8_t *data,
    uint16_t length
);

typedef HAL_StatusTypeDef (*XBee_VerifyBinCallback_t)(
    const uint8_t *data,
    uint16_t length
);

/* ---------------- Handle ---------------- */

typedef struct
{
    UART_HandleTypeDef *huart;

    XBee_GetRpmCallback_t get_rpm;
    XBee_WriteBinCallback_t write_bin;
    XBee_VerifyBinCallback_t verify_bin;

    uint8_t rx_byte;
    uint8_t rx_buffer[XBEE_RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;

    uint32_t rx_errors;

} XBee_ECU_Handle_t;

/* ---------------- API ---------------- */

HAL_StatusTypeDef XBee_ECU_Init(
    XBee_ECU_Handle_t *xbee,
    UART_HandleTypeDef *huart,
    XBee_GetRpmCallback_t get_rpm,
    XBee_WriteBinCallback_t write_bin,
    XBee_VerifyBinCallback_t verify_bin
);

/*
 * Call from HAL_UART_RxCpltCallback().
 */
void XBee_ECU_RxCpltCallback(
    XBee_ECU_Handle_t *xbee,
    UART_HandleTypeDef *huart
);

/*
 * Call periodically from the main loop/task.
 * This function processes incoming XBee commands.
 *
 * The example uses a 1-byte interrupt RX and command timeout.
 */
void XBee_ECU_Process(
    XBee_ECU_Handle_t *xbee
);

/*
 * Call from HAL_UART_ErrorCallback().
 */
void XBee_ECU_ErrorCallback(
    XBee_ECU_Handle_t *xbee,
    UART_HandleTypeDef *huart
);

#ifdef __cplusplus
}
#endif

#endif
