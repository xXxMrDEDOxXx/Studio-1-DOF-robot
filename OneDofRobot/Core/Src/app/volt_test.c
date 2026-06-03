/*
 * volt_test.c — Open-loop voltage excitation (Lab 1 parameter estimation)
 *  ดู volt_test.h สำหรับ waveform + register map
 */
#include "volt_test.h"
#include "base_system.h"
#include "cascade_control.h"   /* Cascade_OpenLoopVolt(), MAX_VOLTAGE */
#include <math.h>

#define VT_PI 3.14159265f

static uint8_t  vt_last_mode = VT_OFF;
static uint32_t vt_tick      = 0;       /* นับ ms ตั้งแต่เริ่ม waveform */

void VoltTest_Init(void)
{
    vt_last_mode = VT_OFF;
    vt_tick      = 0;
}

void VoltTest_Update(void)
{
    uint8_t mode = (uint8_t)modbus_registers[REG_VT_MODE];
    if (mode == VT_OFF) { vt_last_mode = VT_OFF; return; }

    /* เปลี่ยน/เริ่ม waveform ใหม่ → reset เวลา */
    if (mode != vt_last_mode) { vt_tick = 0; vt_last_mode = mode; }

    float t     = (float)vt_tick * 0.001f;
    float amp   = (float)(int16_t)modbus_registers[REG_VT_AMP]    / 100.0f;   /* V   */
    float off   = (float)(int16_t)modbus_registers[REG_VT_OFFSET] / 100.0f;   /* V   */
    float f0    = (float)modbus_registers[REG_VT_F0] / 100.0f;                /* Hz  */
    float f1    = (float)modbus_registers[REG_VT_F1] / 100.0f;                /* Hz  */
    float dur   = (float)modbus_registers[REG_VT_DUR] / 1000.0f;              /* s   */
    uint16_t ns = modbus_registers[REG_VT_STEPS];
    if (dur < 0.001f) dur = 1.0f;

    float V = off;
    switch (mode) {
        case VT_STEP:
            if (t >= dur) { modbus_registers[REG_VT_MODE] = VT_OFF; V = 0.0f; break; }
            V = off + amp;
            break;

        case VT_SINE:
            if (t >= dur) { modbus_registers[REG_VT_MODE] = VT_OFF; V = 0.0f; break; }
            V = off + amp * sinf(2.0f * VT_PI * f0 * t);
            break;

        case VT_CHIRP: {
            if (t >= dur) {                       /* sweep จบ → หยุด */
                modbus_registers[REG_VT_MODE] = VT_OFF;
                V = 0.0f;
                break;
            }
            float k  = (f1 - f0) / dur;           /* Hz/s */
            float ph = 2.0f * VT_PI * (f0 * t + 0.5f * k * t * t);
            V = off + amp * sinf(ph);
            break;
        }

        case VT_STAIR: {
            if (ns < 1) ns = 1;
            float seg = dur / (float)ns;
            uint16_t i = (uint16_t)(t / seg);
            if (i >= ns) {                        /* ครบทุกขั้น → หยุด */
                modbus_registers[REG_VT_MODE] = VT_OFF;
                V = 0.0f;
                break;
            }
            float denom = (ns > 1) ? (float)(ns - 1) : 1.0f;
            V = off + amp * ((float)i / denom);
            break;
        }

        default:
            modbus_registers[REG_VT_MODE] = VT_OFF;
            V = 0.0f;
            break;
    }

    if (V >  MAX_VOLTAGE) V =  MAX_VOLTAGE;
    if (V < -MAX_VOLTAGE) V = -MAX_VOLTAGE;

    Cascade_OpenLoopVolt(V);   /* apply + run KF observer + telemetry + 1kHz logger */
    vt_tick++;
}
