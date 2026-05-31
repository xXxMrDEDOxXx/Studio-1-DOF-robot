/*
 * base_system.c
 *
 *  Created on: 15 พ.ค. 2569
 *      Author: POND
 */


#include "base_system.h"
#include "main.h"    /* HAL_GetTick(), huart2 (extern ใน main.h) */
#include <string.h>  /* memcpy() */

// ตัวแปรเก็บสถานะภายในไฟล์นี้เท่านั้น

static uint32_t last_heartbeat_time = 0;
static bool is_connected = false;
volatile uint16_t modbus_registers[MODBUS_REG_COUNT] = {0};

/* ── FC06 echo detection ───────────────────────────────────────────────────
 * เก็บ response ล่าสุดที่ firmware ส่งออก (FC06 = echo request กลับ)
 * RX callback จะเปรียบ content + เวลา — ถ้า echo ไม่มาใน 50 ms
 * echo_valid ถูก expire อัตโนมัติใน Heartbeat_Update()
 * ─────────────────────────────────────────────────────────────────────── */
uint8_t          modbus_echo_buf[8] = {0};   /* FC06 response ที่เพิ่งส่งออก */
volatile uint8_t  modbus_echo_valid  = 0;    /* 1 = รอ echo อยู่, 0 = ไม่มี  */
volatile uint32_t modbus_echo_time   = 0;    /* HAL_GetTick() ตอนที่ set echo_valid */

/* ── TX buffer สำหรับ non-blocking response (DMA/IT) ──────────────────────────
 *  ต้องเป็น static (persist ระหว่าง IT transmit ที่ทำงานเบื้องหลัง)
 *  ส่งเฉพาะตอน TX ว่าง (gState READY) → ไม่ block RX → write frame ไม่หาย
 *  ── นี่คือ fix ของ heartbeat error: เดิม HAL_UART_Transmit (blocking)
 *     block RX ใน ISR ตอนตอบ read → write (HI) ที่มาแทรกโดน overrun ทิ้ง ── */
static uint8_t mb_tx[135];


// ฟังก์ชันเริ่มต้น (เรียกครั้งเดียวก่อนเข้า while loop)
void Heartbeat_Init(void) {
    // ใส่ค่า YA เริ่มต้นลงไปใน Register 0 รอไว้เลย
    modbus_registers[HEARTBEAT_REG_ADDR] = ROBOT_SAYS_YA;
    last_heartbeat_time = HAL_GetTick();
    is_connected = false;
}

// ฟังก์ชันอัปเดต (เรียกซ้ำๆ ใน while(1) loop)
void Heartbeat_Update(void) {
    uint32_t current_time = HAL_GetTick();

    /* ── 0. Auto-expire echo_valid ────────────────────────────────────────
     * ถ้า echo ไม่มาใน 50 ms (VCP loopback ช้า / UART error ระหว่างรับ)
     * echo_valid จะค้างอยู่ที่ 1 → heartbeat write ถัดไปที่ content เดิม
     * ถูก drop ว่าเป็น echo ผิดๆ → base system timeout เดี๋ยวดับเดี๋ยวติด
     * แก้: expire หลัง 50 ms เพื่อให้ request ถัดไปผ่านได้
     * ─────────────────────────────────────────────────────────────────── */
    if (modbus_echo_valid && (current_time - modbus_echo_time) > 50U) {
        modbus_echo_valid = 0;
    }

    /* ── 1. ตรวจสอบว่า PC เขียน HI ลงมาทับใน Register 0 หรือไม่ ── */
    if (modbus_registers[HEARTBEAT_REG_ADDR] == PC_SAYS_HI) {
        is_connected = true;
        last_heartbeat_time = current_time;
        /* เขียน YA กลับ — base system อ่านเห็น YA = STM32 alive */
        modbus_registers[HEARTBEAT_REG_ADDR] = ROBOT_SAYS_YA;
    }

    /* ── 2. Timeout — base system ค้าง หรือสายหลุด ── */
    if ((current_time - last_heartbeat_time) > HEARTBEAT_TIMEOUT) {
        is_connected = false;
        /* คืน YA เพื่อให้ base system ที่ reconnect เห็น YA แล้วส่ง HI ใหม่ */
        modbus_registers[HEARTBEAT_REG_ADDR] = ROBOT_SAYS_YA;
    }
}

// ฟังก์ชันสำหรับให้ระบบอื่น (เช่น ระบบขับมอเตอร์) เช็คว่าเน็ตหลุดไหม
bool Heartbeat_IsConnected(void) {
    return is_connected;
}


// ฟังก์ชันคำนวณ CRC-16 แบบมาตรฐาน Modbus
static uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ฟังก์ชันแกะแพ็กเกจและตอบกลับ
void Modbus_Parse_Frame(uint8_t *frame, uint16_t length) {
    // 1. เช็คความยาวขั้นต่ำ และ Slave ID ว่าตรงกับบอร์ดเราไหม
    if (length < 8) return;
    if (frame[0] != SLAVE_ID) return;

    // 2. ตรวจสอบความถูกต้องของข้อมูล (CRC)
    // Modbus ส่ง CRC สลับหน้าหลัง (Low Byte มาก่อน High Byte)
    uint16_t crc_received = (frame[length - 1] << 8) | frame[length - 2];
    uint16_t crc_calculated = Modbus_CRC16(frame, length - 2);
    if (crc_received != crc_calculated) return; // ข้อมูลพังระหว่างทาง ให้ทิ้งไป



    uint8_t function_code = frame[1];


    // ---------------------------------------------------------
    // กรณีที่ 1: PC ขออ่านข้อมูล (Function 0x03)
    // ---------------------------------------------------------
    if (function_code == 0x03) {
        uint16_t start_addr = (frame[2] << 8) | frame[3];
        uint16_t num_regs = (frame[4] << 8) | frame[5];



        // ป้องกันการอ่านเกิน Array ที่เรามี
        if (num_regs > 60) return;
        if (start_addr + num_regs > MODBUS_REG_COUNT) return;

        /* TX ยัง busy → ข้าม response นี้ (base จะ retry read เอง)
         * เช็คก่อนเขียน mb_tx เพื่อไม่ทับ buffer ของ TX ที่กำลังส่ง */
        if (huart2.gState != HAL_UART_STATE_READY) return;

        mb_tx[0] = SLAVE_ID;
        mb_tx[1] = 0x03;
        mb_tx[2] = num_regs * 2; // จำนวน Byte ของข้อมูล (1 Register = 2 Bytes)

        uint8_t idx = 3;
        for(int i = 0; i < num_regs; i++) {
            uint16_t val = modbus_registers[start_addr + i];
            mb_tx[idx++] = (val >> 8) & 0xFF;   // Big-Endian
            mb_tx[idx++] = val & 0xFF;
        }

        uint16_t crc_tx = Modbus_CRC16(mb_tx, idx);
        mb_tx[idx++] = crc_tx & 0xFF;         // Low Byte
        mb_tx[idx++] = (crc_tx >> 8) & 0xFF;  // High Byte

        /* non-blocking: ส่งเบื้องหลัง → RX รับ frame ถัดไปได้ระหว่างส่ง */
        HAL_UART_Transmit_IT(&huart2, mb_tx, idx);
    }
    // ---------------------------------------------------------
    // ---------------------------------------------------------
    // กรณีที่ 2: PC สั่งเขียน 1 Register (Function 0x06)
    // ---------------------------------------------------------
    else if (function_code == 0x06) {
        uint16_t reg_addr = (frame[2] << 8) | frame[3];
        uint16_t reg_val  = (frame[4] << 8) | frame[5];

        /* เขียน register เสมอ (สำคัญ: HI/heartbeat + config ต้องลงแม้ TX busy) */
        if (reg_addr < MODBUS_REG_COUNT) {
            modbus_registers[reg_addr] = reg_val;
        }
        /* ตอบ echo เฉพาะตอน TX ว่าง (FC06 response = frame เดิม) */
        if (huart2.gState == HAL_UART_STATE_READY) {
            memcpy(mb_tx, frame, 8);
            memcpy(modbus_echo_buf, frame, 8);   /* echo filter tracking */
            modbus_echo_time  = HAL_GetTick();
            modbus_echo_valid = 1;
            HAL_UART_Transmit_IT(&huart2, mb_tx, 8);
        }
    }
    // ---------------------------------------------------------
    // กรณีที่ 3: PC สั่งเขียนหลาย Register (Function 0x10 = FC16)
    // ---------------------------------------------------------
    else if (function_code == 0x10) {
        uint16_t start_addr = (frame[2] << 8) | frame[3];
        uint16_t num_regs   = (frame[4] << 8) | frame[5];
        // frame[6] = byte count = num_regs * 2

        /* เขียน register เสมอ (ลงแม้ TX busy) */
        if (start_addr + num_regs <= MODBUS_REG_COUNT) {
            for (uint16_t i = 0; i < num_regs; i++) {
                uint16_t val = (frame[7 + i * 2] << 8) | frame[8 + i * 2];
                modbus_registers[start_addr + i] = val;
            }
        }

        /* ตอบ: [ID][0x10][start_hi][start_lo][num_hi][num_lo][CRC_lo][CRC_hi]
         * เฉพาะตอน TX ว่าง */
        if (huart2.gState == HAL_UART_STATE_READY) {
            mb_tx[0] = SLAVE_ID;
            mb_tx[1] = 0x10;
            mb_tx[2] = frame[2];
            mb_tx[3] = frame[3];
            mb_tx[4] = frame[4];
            mb_tx[5] = frame[5];
            uint16_t crc = Modbus_CRC16(mb_tx, 6);
            mb_tx[6] = crc & 0xFF;
            mb_tx[7] = (crc >> 8) & 0xFF;
            HAL_UART_Transmit_IT(&huart2, mb_tx, 8);
        }
    }
}
