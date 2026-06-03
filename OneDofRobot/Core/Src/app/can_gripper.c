/*
 * can_gripper.c
 *
 * CAN 2.0A master driver for ProtocolSpec.pdf v1.0.1.
 * Sends:
 *   0x600 [0x05] every 500 ms                      Master heartbeat
 *   0x410 [0x40,0x00,0x05,0x00] once after startup  Force node operational
 *   0x210 [0x10,0x00,relay_mask] on output changes  Relay bank write
 *   0x210 [0x20,0x10] periodically/on request       Opto bank read
 */

#include "can_gripper.h"

#define CAN_GRIPPER_INSTR_WRITE_REQ   0x10U
#define CAN_GRIPPER_INSTR_READ_REQ    0x20U
#define CAN_GRIPPER_INSTR_WRITE_ACK   0x11U
#define CAN_GRIPPER_INSTR_READ_RESP   0x21U
#define CAN_GRIPPER_INSTR_WRITE_CFG   0x40U

#define CAN_GRIPPER_PARAM_NMT         0x00U
#define CAN_GRIPPER_HEARTBEAT_MS      500U
#define CAN_GRIPPER_OPTO_POLL_MS      100U
#define CAN_GRIPPER_RX_DRAIN_LIMIT    8U

static volatile uint8_t  g_started         = 0;
static volatile uint8_t  g_desired_mask    = CAN_GRIPPER_RELAY_UP | CAN_GRIPPER_RELAY_OPEN;
static volatile uint8_t  g_actual_mask     = 0;
static volatile uint8_t  g_pending_mask    = 1;
static volatile uint8_t  g_opto_mask       = 0;
static volatile uint8_t  g_opto_valid      = 0;
static volatile uint8_t  g_node_state      = CAN_GRIPPER_NODE_BOOTING;
static volatile uint8_t  g_emcy_code       = 0;
static volatile uint8_t  g_config_pending  = 1;
static volatile uint8_t  g_opto_read_req   = 1;

static volatile uint32_t g_last_hb_ms      = 0;
static volatile uint32_t g_last_opto_ms    = 0;
static volatile uint32_t g_last_rx_ms      = 0;
static volatile uint32_t g_tx_ok_count     = 0;
static volatile uint32_t g_tx_fail_count   = 0;
static volatile uint32_t g_rx_count        = 0;

static uint8_t dlc_to_len(uint32_t dlc)
{
    return (dlc <= FDCAN_DLC_BYTES_8) ? (uint8_t)dlc : 8U;
}

static HAL_StatusTypeDef can_send(uint32_t identifier, uint32_t dlc, const uint8_t *data)
{
    if (!g_started || HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        g_tx_fail_count++;
        return HAL_BUSY;
    }

    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = identifier;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = dlc;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, data);
    if (st == HAL_OK) {
        g_tx_ok_count++;
    } else {
        g_tx_fail_count++;
    }
    return st;
}

static HAL_StatusTypeDef send_master_heartbeat(void)
{
    const uint8_t data[1] = { CAN_GRIPPER_MASTER_OPER };
    return can_send(CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_MHB, CAN_GRIPPER_BROADCAST_ID),
                    FDCAN_DLC_BYTES_1,
                    data);
}

static HAL_StatusTypeDef send_force_operational(void)
{
    const uint8_t data[4] = {
        CAN_GRIPPER_INSTR_WRITE_CFG,
        CAN_GRIPPER_PARAM_NMT,
        CAN_GRIPPER_NODE_OPER,
        0x00U
    };
    return can_send(CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_CFG_REQ, CAN_GRIPPER_NODE_ID),
                    FDCAN_DLC_BYTES_4,
                    data);
}

static HAL_StatusTypeDef send_relay_mask(uint8_t relay_mask)
{
    const uint8_t data[3] = {
        CAN_GRIPPER_INSTR_WRITE_REQ,
        CAN_GRIPPER_TARGET_RELAY,
        (uint8_t)(relay_mask & CAN_GRIPPER_RELAY_MASK)
    };
    return can_send(CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_CMD_REQ, CAN_GRIPPER_NODE_ID),
                    FDCAN_DLC_BYTES_3,
                    data);
}

static HAL_StatusTypeDef send_opto_read(void)
{
    const uint8_t data[2] = {
        CAN_GRIPPER_INSTR_READ_REQ,
        CAN_GRIPPER_TARGET_OPTO
    };
    return can_send(CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_CMD_REQ, CAN_GRIPPER_NODE_ID),
                    FDCAN_DLC_BYTES_2,
                    data);
}

static void handle_rx(const FDCAN_RxHeaderTypeDef *rx, const uint8_t *data)
{
    if (rx->IdType != FDCAN_STANDARD_ID || rx->RxFrameType != FDCAN_DATA_FRAME) {
        return;
    }

    uint8_t len = dlc_to_len(rx->DataLength);
    g_rx_count++;
    g_last_rx_ms = HAL_GetTick();

    switch (rx->Identifier) {
        case CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_RT_DATA, CAN_GRIPPER_NODE_ID):
            if (len >= 1U) {
                g_opto_mask  = data[0];
                g_opto_valid = 1;
            }
            break;

        case CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_CMD_RESP, CAN_GRIPPER_NODE_ID):
            if (len >= 3U && data[0] == CAN_GRIPPER_INSTR_WRITE_ACK &&
                data[1] == CAN_GRIPPER_TARGET_RELAY) {
                g_actual_mask = data[2];
            } else if (len >= 3U && data[0] == CAN_GRIPPER_INSTR_READ_RESP &&
                       data[1] == CAN_GRIPPER_TARGET_OPTO) {
                g_opto_mask  = data[2];
                g_opto_valid = 1;
            }
            break;

        case CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_CFG_RESP, CAN_GRIPPER_NODE_ID):
            g_config_pending = 0;
            break;

        case CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_NHB, CAN_GRIPPER_NODE_ID):
            if (len >= 1U) {
                g_node_state = data[0];
            }
            break;

        case CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_EMCY, CAN_GRIPPER_NODE_ID):
            if (len >= 1U) {
                g_emcy_code = data[0];
            }
            break;

        default:
            break;
    }
}

static void drain_rx_fifo(void)
{
    uint8_t drained = 0;

    while (drained < CAN_GRIPPER_RX_DRAIN_LIMIT &&
           HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef rx = {0};
        uint8_t data[8] = {0};
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
            break;
        }
        handle_rx(&rx, data);
        drained++;
    }
}

static void config_filter(uint8_t filter_index, uint32_t identifier)
{
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = filter_index;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = identifier;
    filter.FilterID2    = 0x7FFU;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        g_tx_fail_count++;
    }
}

void CanGripper_Init(void)
{
    g_started        = 0;
    g_desired_mask   = CAN_GRIPPER_RELAY_UP | CAN_GRIPPER_RELAY_OPEN;
    g_actual_mask    = 0;
    g_pending_mask   = 1;
    g_opto_mask      = 0;
    g_opto_valid     = 0;
    g_node_state     = CAN_GRIPPER_NODE_BOOTING;
    g_emcy_code      = 0;
    g_config_pending = 1;
    g_opto_read_req  = 1;
    uint32_t now = HAL_GetTick();
    g_last_hb_ms     = now - CAN_GRIPPER_HEARTBEAT_MS;
    g_last_opto_ms   = now - CAN_GRIPPER_OPTO_POLL_MS;
    g_last_rx_ms     = 0;

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        g_tx_fail_count++;
        return;
    }

    config_filter(0, CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_EMCY,     CAN_GRIPPER_NODE_ID));
    config_filter(1, CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_RT_DATA,  CAN_GRIPPER_NODE_ID));
    config_filter(2, CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_CMD_RESP, CAN_GRIPPER_NODE_ID));
    config_filter(3, CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_CFG_RESP, CAN_GRIPPER_NODE_ID));
    config_filter(4, CAN_GRIPPER_MAKE_ID(CAN_GRIPPER_FUNC_NHB,      CAN_GRIPPER_NODE_ID));

    if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK) {
        g_started = 1;
    } else {
        g_tx_fail_count++;
    }
}

void CanGripper_Update(void)
{
    if (!g_started) {
        return;
    }

    uint32_t now = HAL_GetTick();

    drain_rx_fifo();

    if ((now - g_last_hb_ms) >= CAN_GRIPPER_HEARTBEAT_MS) {
        if (send_master_heartbeat() == HAL_OK) {
            g_last_hb_ms = now;
        }
    }

    if (g_config_pending) {
        if (send_force_operational() == HAL_OK) {
            g_config_pending = 0;
        }
    }

    if (g_pending_mask) {
        uint8_t mask = g_desired_mask;
        if (send_relay_mask(mask) == HAL_OK) {
            g_actual_mask  = mask;
            g_pending_mask = 0;
        }
    }

    if (g_opto_read_req ||
        (now - g_last_opto_ms) >= CAN_GRIPPER_OPTO_POLL_MS) {
        if (send_opto_read() == HAL_OK) {
            g_opto_read_req = 0;
            g_last_opto_ms  = now;
        }
    }
}

void CanGripper_SetRelayMask(uint8_t relay_mask)
{
    relay_mask &= CAN_GRIPPER_RELAY_MASK;
    if (relay_mask != g_desired_mask) {
        g_desired_mask = relay_mask;
        g_pending_mask = 1;
    }
}

void CanGripper_SetArmDown(uint8_t down)
{
    uint8_t mask = g_desired_mask;
    if (down) {
        mask = (uint8_t)((mask | CAN_GRIPPER_RELAY_DOWN) & (uint8_t)~CAN_GRIPPER_RELAY_UP);
    } else {
        mask = (uint8_t)((mask | CAN_GRIPPER_RELAY_UP) & (uint8_t)~CAN_GRIPPER_RELAY_DOWN);
    }
    CanGripper_SetRelayMask(mask);
}

void CanGripper_SetJawClose(uint8_t close)
{
    uint8_t mask = g_desired_mask;
    if (close) {
        mask = (uint8_t)((mask | CAN_GRIPPER_RELAY_CLOSE) & (uint8_t)~CAN_GRIPPER_RELAY_OPEN);
    } else {
        mask = (uint8_t)((mask | CAN_GRIPPER_RELAY_OPEN) & (uint8_t)~CAN_GRIPPER_RELAY_CLOSE);
    }
    CanGripper_SetRelayMask(mask);
}

void CanGripper_RequestOptoRead(void)
{
    g_opto_read_req = 1;
}

uint8_t CanGripper_GetDesiredMask(void)       { return g_desired_mask; }
uint8_t CanGripper_GetActualMask(void)        { return g_actual_mask; }
uint8_t CanGripper_GetOptoMask(void)          { return g_opto_mask; }
uint8_t CanGripper_GetNodeState(void)         { return g_node_state; }
uint8_t CanGripper_IsStarted(void)            { return g_started; }
uint8_t CanGripper_HasOptoFeedback(void)      { return g_opto_valid; }
uint8_t CanGripper_GetEmergencyCode(void)     { return g_emcy_code; }
uint32_t CanGripper_GetTxOkCount(void)        { return g_tx_ok_count; }
uint32_t CanGripper_GetTxFailCount(void)      { return g_tx_fail_count; }
uint32_t CanGripper_GetRxCount(void)          { return g_rx_count; }
