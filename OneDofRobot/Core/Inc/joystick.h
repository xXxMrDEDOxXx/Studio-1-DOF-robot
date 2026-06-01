/*
 * joystick.h
 *
 *  Funduino Joystick Shield — Manual Mode Interface
 * ─────────────────────────────────────────────────────────────────────────────
 *  Buttons (active LOW, pull-up):
 *    A → PA5  : Emergency stop / Reset (TOGGLE: 1st press = E-stop, 2nd = reset;
 *               cabinet PC13 button also clears)
 *    B → PA6  : Gripper Pick/Place toggle
 *    C → PA7  : Set Home (zero encoder at current position)
 *    D → PB11 : Gripper arm Up/Down toggle
 *    K → PB10 : Toggle movement mode (Free ↔ Point-to-Point)
 *
 *  ADC X-axis → PC3 (ADC2_IN9), 0–4095:
 *    Free  mode: < 800 → CCW at 15% duty | > 3500 → CW at 15% duty
 *    Point mode: < 800 → +5° (CCW) move  | > 3500 → −5° (CW) move
 *                (S-curve / Septic, jerk-continuous; re-trigger after neutral)
 *
 *  ⚠ Only active when current_system_mode == MODE_MANUAL
 *
 *  Direction defines (flip if CCW/CW is reversed):
 *    JOY_DIR_CCW — GPIO level for CCW on Motor_Dir_Pin
 *    JOY_DIR_CW  — GPIO level for CW  on Motor_Dir_Pin
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <stdint.h>

/* ── Motor direction ─────────────────────────────────────────────────────────
 * วัดจริง: Motor_Dir=RESET → หมุน CW. ดังนั้น CCW = SET
 * (ถ้าหุ่นยังกลับด้าน สลับ 2 บรรทัดนี้ + เปลี่ยน BS_DIR_SIGN ใน base_system.h) */
#define JOY_DIR_CCW   GPIO_PIN_SET     /* Motor_Dir_Pin level for CCW */
#define JOY_DIR_CW    GPIO_PIN_RESET   /* Motor_Dir_Pin level for CW  */

/* ── PWM duty for free-movement mode: 15% of ARR_MAX (9999) ─────────────── */
#define JOY_FREE_DUTY   1500U

/* ── ADC thresholds (0–4095) ─────────────────────────────────────────────── */
#define JOY_ADC_LOW   800U    /* below this → CCW */
#define JOY_ADC_HIGH  3500U   /* above this → CW  */

/* ── Step size for point-to-point mode ───────────────────────────────────── */
#define JOY_STEP_DEG  5.0f    /* degrees per joystick click */

/* ── S-curve (Septic) move time per click [s] ────────────────────────────────
 * เวลาเคลื่อนที่ต่อคลิก 5° (จะถูกยืดอัตโนมัติถ้า v/a เกิน hardware cap)
 * เล็กลง = jog ไว/กระชากขึ้น | ใหญ่ขึ้น = เนียน/ช้าลง */
#define JOY_POINT_MOVE_TIME  0.5f

/* ── Debounce time [ms] ──────────────────────────────────────────────────── */
#define JOY_DEBOUNCE_MS  20U

/* ── Movement modes ──────────────────────────────────────────────────────── */
typedef enum {
    JOY_MODE_FREE  = 0,   /* free movement (direct 15% duty) */
    JOY_MODE_POINT = 1,   /* point-to-point (±5° per click)  */
} JoyMode_t;

/* ── Public API ──────────────────────────────────────────────────────────── */
void    Joystick_Init(void);

/* Call from TIM6 ISR inside MODE_MANUAL section.
 * Returns 1 if joystick is actively controlling the motor (caller should skip
 * Dashboard_Update to avoid conflict). Returns 0 otherwise. */
uint8_t Joystick_Update(void);

/* Query current joystick movement mode (for UI/debug) */
JoyMode_t Joystick_GetMode(void);

#endif /* JOYSTICK_H_ */
