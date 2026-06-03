/*
 * can_gripper.h
 *
 * CAN 2.0A master driver for the external gripper relay node.
 * ProtocolSpec.pdf v1.0.1: Node ID 0x10, 500 kbps, relay bank command.
 */

#ifndef CAN_GRIPPER_H_
#define CAN_GRIPPER_H_

#include "main.h"
#include <stdint.h>

#define CAN_GRIPPER_NODE_ID          0x10U
#define CAN_GRIPPER_BROADCAST_ID     0x00U
#define CAN_GRIPPER_MAKE_ID(func, node) \
    ((((uint32_t)(func) & 0x07U) << 8) | ((uint32_t)(node) & 0xFFU))

#define CAN_GRIPPER_FUNC_EMCY        0x0U
#define CAN_GRIPPER_FUNC_RT_DATA     0x1U
#define CAN_GRIPPER_FUNC_CMD_REQ     0x2U
#define CAN_GRIPPER_FUNC_CMD_RESP    0x3U
#define CAN_GRIPPER_FUNC_CFG_REQ     0x4U
#define CAN_GRIPPER_FUNC_CFG_RESP    0x5U
#define CAN_GRIPPER_FUNC_MHB         0x6U
#define CAN_GRIPPER_FUNC_NHB         0x7U

#define CAN_GRIPPER_TARGET_RELAY     0x00U
#define CAN_GRIPPER_TARGET_OPTO      0x10U

#define CAN_GRIPPER_RELAY_UP         0x01U
#define CAN_GRIPPER_RELAY_DOWN       0x02U
#define CAN_GRIPPER_RELAY_CLOSE      0x04U
#define CAN_GRIPPER_RELAY_OPEN       0x08U
#define CAN_GRIPPER_RELAY_MASK       0x0FU

#define CAN_GRIPPER_MASTER_STOPPED   0x00U
#define CAN_GRIPPER_MASTER_OPER      0x05U
#define CAN_GRIPPER_NODE_BOOTING     0x00U
#define CAN_GRIPPER_NODE_OPER        0x05U
#define CAN_GRIPPER_NODE_FAILSAFE    0xFFU

void     CanGripper_Init(void);
void     CanGripper_Update(void);

void     CanGripper_SetRelayMask(uint8_t relay_mask);
void     CanGripper_SetArmDown(uint8_t down);
void     CanGripper_SetJawClose(uint8_t close);
void     CanGripper_RequestOptoRead(void);

uint8_t  CanGripper_GetDesiredMask(void);
uint8_t  CanGripper_GetActualMask(void);
uint8_t  CanGripper_GetOptoMask(void);
uint8_t  CanGripper_GetNodeState(void);
uint8_t  CanGripper_IsStarted(void);
uint8_t  CanGripper_HasOptoFeedback(void);
uint8_t  CanGripper_GetEmergencyCode(void);

uint32_t CanGripper_GetTxOkCount(void);
uint32_t CanGripper_GetTxFailCount(void);
uint32_t CanGripper_GetRxCount(void);

#endif /* CAN_GRIPPER_H_ */
