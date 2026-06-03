#include "datalog.h"
#include "base_system.h"   /* modbus_registers[], REG_LOG_* (LIMIT/TRIG/MODE) */
#include <math.h>          /* fabsf() */

/* buffer flat ก้อนเดียว ใช้ร่วมสองโหมด (full 7ch×2000 หรือ long 2ch×20000) */
static int16_t           g_buf[LOG_FLAT];
static volatile uint16_t g_idx   = 0;
static volatile uint16_t g_limit = LOG_N;   /* จำนวน sample ที่จะอัด */
static volatile uint8_t  g_chans = LOG_C;   /* 7 = full, 2 = long (V,qd) */
static volatile uint8_t  g_state = 0;       /* 0=idle 1=wait_trig 2=capturing 3=done */
static volatile uint8_t  g_trig_mode = 0;   /* 0=Immediate 1=Wait Ref Pos 2=Wait Voltage */
static float             g_trig_baseline = 0;
static uint8_t           g_trig_ready = 0;

static inline int16_t sat16(float x) {
    if (x >  32767.0f) return  32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)x;
}

void DataLog_Init(void) {
    g_idx = 0; g_state = 0; g_chans = LOG_C; g_limit = LOG_N;
}

void DataLog_Arm(void) {
    g_idx = 0;
    g_trig_ready = 0;
    g_state = (g_trig_mode == 0) ? 2 : 1;   /* immediate → อัดเลย, ไม่งั้นรอ trigger */
}

/* เรียกทุก control tick (1 kHz) */
void DataLog_Sample(float ref_q, float q, float ref_qd, float qd,
                    float ref_qdd, float V, float i)
{
    if (g_state == 0 || g_state == 3) return;

    /* State 1: รอสัญญาณ trigger ก่อนอัดจริง (sync ตอน signal เริ่ม) */
    if (g_state == 1) {
        float current_val = (g_trig_mode == 1) ? ref_q : V;
        if (!g_trig_ready) {
            g_trig_baseline = current_val;
            g_trig_ready = 1;
            return;
        }
        if (fabsf(current_val - g_trig_baseline) > 0.001f) {
            g_state = 2;            /* signal มาแล้ว → เริ่มอัด */
        } else {
            return;
        }
    }

    /* State 2: กำลังอัด — เก็บแบบ flat ตามจำนวน channel ของโหมด */
    if (g_state == 2) {
        uint32_t base = (uint32_t)g_idx * g_chans;
        if (g_chans == LOG_C_LONG) {            /* long: V, qd */
            g_buf[base + 0] = sat16(V  * 1000.0f);
            g_buf[base + 1] = sat16(qd * 1000.0f);
        } else {                                /* full: 7 channels */
            g_buf[base + 0] = sat16(ref_q   * 1000.0f);
            g_buf[base + 1] = sat16(q       * 1000.0f);
            g_buf[base + 2] = sat16(ref_qd  * 1000.0f);
            g_buf[base + 3] = sat16(qd      * 1000.0f);
            g_buf[base + 4] = sat16(ref_qdd * 100.0f);
            g_buf[base + 5] = sat16(V       * 1000.0f);
            g_buf[base + 6] = sat16(i       * 1000.0f);
        }
        if (++g_idx >= g_limit) g_state = 3;    /* ครบ → done */
    }
}

void DataLog_Service(void)
{
    uint16_t ctrl = modbus_registers[REG_LOG_CTRL];
    if (ctrl == 1U) {                           /* arm/start */
        modbus_registers[REG_LOG_CTRL] = 0;

        uint8_t  longm = (modbus_registers[REG_LOG_MODE] != 0) ? 1 : 0;
        g_chans = longm ? LOG_C_LONG : LOG_C;
        uint16_t maxn  = longm ? LOG_N_LONG : LOG_N;

        uint16_t limit = modbus_registers[REG_LOG_LIMIT];
        g_limit = (limit > 0 && limit <= maxn) ? limit : maxn;
        g_trig_mode = (uint8_t)modbus_registers[REG_LOG_TRIG];

        DataLog_Arm();
    } else if (ctrl == 2U) {                    /* abort */
        modbus_registers[REG_LOG_CTRL] = 0;
        g_state = 0; g_idx = 0;
    }

    /* status → PC: 0=idle 1=capturing/waiting 2=done */
    modbus_registers[REG_LOG_STATUS] = (g_state == 3) ? 2 : (g_state > 0 ? 1 : 0);
    modbus_registers[REG_LOG_COUNT]  = g_idx;
}

uint16_t DataLog_Total(void)
{
    return (uint16_t)LOG_FLAT;                  /* 40000 < 65535 — ขอบสำหรับ readout */
}

uint16_t DataLog_ReadFlat(uint32_t idx)
{
    if (idx >= (uint32_t)LOG_FLAT) return 0;
    return (uint16_t)g_buf[idx];
}
