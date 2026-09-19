#include "xbee_ecu.h"
#include "ecu_data.h"
#include <string.h>

extern CRC_HandleTypeDef hcrc;

/*
 * NOTE:
 * This driver deliberately performs an independent engine-RPM check
 * inside the MCU. The PC-side check is NOT considered a safety barrier.
 */

#define RX_TIMEOUT_MS             3000U
#define MAX_FRAME_DATA            XBEE_BIN_SIZE



static HAL_StatusTypeDef uart_send(
    XBee_ECU_Handle_t *xbee,
    const uint8_t *data,
    uint16_t len
)
{
    return HAL_UART_Transmit(
        xbee->huart,
        (uint8_t *)data,
        len,
        1000U
    );
}

static void send_bin_ack(
    XBee_ECU_Handle_t *xbee,
    uint8_t status
)
{
    uint8_t packet[4];

    packet[0] = XBEE_SOF1;
    packet[1] = XBEE_SOF2;
    packet[2] = XBEE_RSP_BIN_ACK;
    packet[3] = status;

    (void)uart_send(xbee, packet, sizeof(packet));
}

static void send_monitor_response(
    XBee_ECU_Handle_t *xbee
)
{
    /*
     * Python expects:
     *
     * byte 0-1 : AA 55
     * byte 2-3 : reserved
     * byte 4-5 : RPM
     * byte 6-7 : Oil Temp
     * byte 8-9 : Throttle
     * byte 10-11: Speed
     */
    uint8_t packet[12];

    int rpm      = 0;
    int oil_temp = 0;
    int throttle = 0;
    int speed    = 0;

    /*
     * Replace these four lines with your actual ECU data.
     *
     * rpm MUST come from the same real-time RPM measurement used
     * by the engine-control logic.
     */

        rpm = rpm_A;


    /*
     * TODO:
     * Replace these with your actual measured values.
     */


    //元のコードの変数に変換
    oil_temp = tmp;
    throttle = THper;
    /* No vehicle-speed input is implemented yet; never report a fabricated value. */
    speed = 0;

    packet[0] = XBEE_SOF1;
    packet[1] = XBEE_SOF2;

    packet[2] = 0x00U;
    packet[3] = 0x00U;

    packet[4] = (uint8_t)(rpm & 0xFFU);
    packet[5] = (uint8_t)(rpm >> 8);

    packet[6] = (uint8_t)(oil_temp & 0xFFU);
    packet[7] = (uint8_t)(oil_temp >> 8);

    packet[8] = (uint8_t)(throttle & 0xFFU);
    packet[9] = (uint8_t)(throttle >> 8);

    packet[10] = (uint8_t)(speed & 0xFFU);
    packet[11] = (uint8_t)(speed >> 8);

    (void)uart_send(xbee, packet, sizeof(packet));
}

static HAL_StatusTypeDef receive_exact(
    XBee_ECU_Handle_t *xbee,
    uint8_t *buf,
    uint16_t len,
    uint32_t timeout_ms
)
{
    uint32_t start = HAL_GetTick();
    uint16_t pos = 0;

    while (pos < len)
    {
        if (xbee->rx_head != xbee->rx_tail)
        {
            buf[pos++] = xbee->rx_buffer[xbee->rx_tail];
            xbee->rx_tail = (xbee->rx_tail + 1U) % XBEE_RX_BUFFER_SIZE;
        }

        if ((HAL_GetTick() - start) > timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef receive_byte(
    XBee_ECU_Handle_t *xbee,
    uint8_t *byte,
    uint32_t timeout_ms
)
{
    return receive_exact(
        xbee,
        byte,
        1U,
        timeout_ms
    );
}

static uint8_t engine_is_stopped(
    XBee_ECU_Handle_t *xbee
)
{
    int rpm;

    if (xbee->get_rpm == NULL)
    {
        /*
         * Fail closed:
         * If RPM cannot be obtained, do NOT permit a BIN update.
         */
        return 0U;
    }

    rpm = xbee->get_rpm();

    /*
     * Exact zero as requested.
     */
    return (rpm == XBEE_ENGINE_STOP_RPM) ? 1U : 0U;
}

static uint16_t XBee_ECU_CRC32(
        const uint8_t *data,
        uint32_t length
    )
    {
        if (data == NULL)
            return 0U;

        if (length > XBEE_BIN_SIZE - 2U)
            return 0U;

        uint32_t crc32 = 0xFFFFFFFFU;
        for (uint32_t i = 0; i < length; i++)
        {
            crc32 ^= data[i];
            for (uint32_t bit = 0; bit < 8U; bit++)
            {
                crc32 = (crc32 & 1U) ? ((crc32 >> 1U) ^ 0xEDB88320U) : (crc32 >> 1U);
            }
        }

        return (uint16_t)((crc32 ^ 0xFFFFFFFFU) & 0xFFFFU);
    }


static void discard_bytes(
    XBee_ECU_Handle_t *xbee,
    uint16_t len
)
{
    uint8_t dummy;

    for (uint16_t i = 0; i < len; i++)
    {
        if (receive_byte(
                xbee,
                &dummy,
                50U) != HAL_OK)
        {
            break;
        }
    }
}

static void handle_bin_write(
    XBee_ECU_Handle_t *xbee,
    uint16_t length
)
{
    static uint8_t bin[XBEE_BIN_SIZE];

    uint16_t received_crc;
    uint16_t calculated_crc;

    /*
     * Safety check #1:
     * Reject before receiving/writing anything if engine is running.
     */
    if (!engine_is_stopped(xbee))
    {
        send_bin_ack(
            xbee,
            XBEE_BIN_ENGINE_RUNNING
        );

        /*
         * The PC already sent the payload.
         * Consume it so the stream is returned to a known state.
         */
        if (length <= XBEE_BIN_SIZE)
        {
            discard_bytes(xbee, length);
        }

        return;
    }

    if (length != XBEE_BIN_SIZE)
    {
        send_bin_ack(
            xbee,
            XBEE_BIN_BAD_LENGTH
        );

        if (length < 4096U)
        {
            discard_bytes(xbee, length);
        }

        return;
    }

    /*
     * Receive the entire BIN into RAM.
     *
     * Nothing is written to EEPROM while receiving.
     */
    if (receive_exact(
            xbee,
            bin,
            XBEE_BIN_SIZE,
            RX_TIMEOUT_MS) != HAL_OK)
    {
        send_bin_ack(
            xbee,
            XBEE_BIN_RX_TIMEOUT
        );
        return;
    }

    /*
     * Safety check #2:
     * Check RPM AGAIN immediately before EEPROM write.
     */
    if (!engine_is_stopped(xbee))
    {
        send_bin_ack(
            xbee,
            XBEE_BIN_ENGINE_RUNNING
        );
        return;
    }

    /*
     * The existing Python BIN contains a 16-bit CRC at the end.
     *
     * IMPORTANT:
     * The original Python code calculates:
     *
     *   zlib.crc32(binary) & 0xFFFF
     *
     * So the exact CRC algorithm is NOT the CRC16-CCITT above.
     *
     * To remain compatible with your existing BIN files, calculate
     * the same lower 16 bits of CRC32 here.
     */


    calculated_crc =
        (uint16_t)(XBee_ECU_CRC32(
            bin,
            XBEE_BIN_SIZE - 2U) & 0xFFFFU);

    received_crc =
        (uint16_t)bin[XBEE_BIN_SIZE - 2U] |
        ((uint16_t)bin[XBEE_BIN_SIZE - 1U] << 8);

    if (calculated_crc != received_crc)
    {
        send_bin_ack(
            xbee,
            XBEE_BIN_CRC_ERROR
        );
        return;
    }

    /*
     * Safety check #3:
     * Final RPM check immediately before the application callback.
     */
    if (!engine_is_stopped(xbee))
    {
        send_bin_ack(
            xbee,
            XBEE_BIN_ENGINE_RUNNING
        );
        return;
    }

    /*
     * Write only after all checks have passed.
     *
     * For a production ECU, write_bin should use a staging/inactive
     * EEPROM slot and switch the active slot only after verification.
     */
    if (xbee->write_bin == NULL ||
        xbee->verify_bin == NULL)
    {
        send_bin_ack(
            xbee,
            XBEE_BIN_EEPROM_ERROR
        );
        return;
    }

    if (xbee->write_bin(
            bin,
            XBEE_BIN_SIZE) != HAL_OK)
    {
        send_bin_ack(
            xbee,
            XBEE_BIN_EEPROM_ERROR
        );
        return;
    }

    /*
     * Read-back verification.
     */
    if (xbee->verify_bin(
            bin,
            XBEE_BIN_SIZE) != HAL_OK)
    {
        send_bin_ack(
            xbee,
            XBEE_BIN_EEPROM_ERROR
        );
        return;
    }

    send_bin_ack(
        xbee,
        XBEE_BIN_OK
    );
}

HAL_StatusTypeDef XBee_ECU_Init(
    XBee_ECU_Handle_t *xbee,
    UART_HandleTypeDef *huart,
    XBee_GetRpmCallback_t get_rpm,
    XBee_WriteBinCallback_t write_bin,
    XBee_VerifyBinCallback_t verify_bin
)
{
    if (xbee == NULL ||
        huart == NULL ||
        get_rpm == NULL ||
        write_bin == NULL ||
        verify_bin == NULL)
    {
        return HAL_ERROR;
    }

    memset(
        xbee,
        0,
        sizeof(XBee_ECU_Handle_t)
    );

    xbee->huart = huart;
    xbee->get_rpm = get_rpm;
    xbee->write_bin = write_bin;
    xbee->verify_bin = verify_bin;

    return HAL_UART_Receive_IT(
        xbee->huart,
        &xbee->rx_byte,
        1U
    );
}

void XBee_ECU_RxCpltCallback(
    XBee_ECU_Handle_t *xbee,
    UART_HandleTypeDef *huart
)
{
    if (xbee == NULL ||
        huart != xbee->huart)
    {
        return;
    }

    uint16_t next = (xbee->rx_head + 1U) % XBEE_RX_BUFFER_SIZE;
    if (next == xbee->rx_tail)
    {
        xbee->rx_errors++;
    }
    else
    {
        xbee->rx_buffer[xbee->rx_head] = xbee->rx_byte;
        xbee->rx_head = next;
    }

    (void)HAL_UART_Receive_IT(xbee->huart, &xbee->rx_byte, 1U);
}

void XBee_ECU_ErrorCallback(
    XBee_ECU_Handle_t *xbee,
    UART_HandleTypeDef *huart
)
{
    if (xbee == NULL ||
        huart != xbee->huart)
    {
        return;
    }

    xbee->rx_errors++;
    xbee->rx_tail = xbee->rx_head;

    (void)HAL_UART_Receive_IT(
        xbee->huart,
        &xbee->rx_byte,
        1U
    );
}

void XBee_ECU_Process(
    XBee_ECU_Handle_t *xbee
)
{
    uint8_t b;
    uint8_t cmd;
    uint8_t len_l;
    uint8_t len_h;
    uint16_t length;

    if (xbee == NULL)
    {
        return;
    }

    /*
     * We wait for AA 55.
     */
    if (xbee->rx_head == xbee->rx_tail)
    {
        return;
    }

    b = xbee->rx_buffer[xbee->rx_tail];
    xbee->rx_tail = (xbee->rx_tail + 1U) % XBEE_RX_BUFFER_SIZE;

    if (b != XBEE_SOF1)
    {
        return;
    }

    if (receive_byte(
            xbee,
            &b,
            100U) != HAL_OK)
    {
        return;
    }

    if (b != XBEE_SOF2)
    {
        return;
    }

    if (receive_byte(
            xbee,
            &cmd,
            100U) != HAL_OK)
    {
        return;
    }

    if (cmd == XBEE_CMD_MONITOR)
    {
        if (receive_byte(
                xbee,
                &b,
                100U) != HAL_OK)
        {
            return;
        }

        /*
         * Monitor request currently has length 0.
         */
        if (b != 0U)
        {
            return;
        }

        send_monitor_response(xbee);
        return;
    }

    if (cmd == XBEE_CMD_WRITE_BIN)
    {
        if (receive_byte(
                xbee,
                &len_l,
                100U) != HAL_OK)
        {
            return;
        }

        if (receive_byte(
                xbee,
                &len_h,
                100U) != HAL_OK)
        {
            return;
        }

        length =
            (uint16_t)len_l |
            ((uint16_t)len_h << 8);

        handle_bin_write(
            xbee,
            length
        );

        return;
    }

    /*
     * Unknown command.
     */
}

