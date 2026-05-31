/*
 * homing.h
 *
 *  State machine: [H_LEAVE →] H_SEEK → H_COUNT → H_RETURN → [H_UNWIND →] H_DONE
 *  เรียกจาก TIM6 ISR ทุก 1 ms (case MODE_HOMING)
 *
 *  H_LEAVE : (ถ้า sensor HIGH ตอน start) ขับออกทิศตรงข้ามจน LOW → เข้า H_SEEK
 *  H_SEEK  : วิ่งจน sensor=HIGH → หยุดทันที → zero encoder
 *            (วัด seek travel จาก start; ถ้า > CPR/2 = หมุนผิดฝั่ง → wrong_side)
 *  H_COUNT : คืบผ่าน zone นับ pulse จน sensor=LOW → zone_count
 *  H_RETURN: ถอยหลัง zone_count/2 pulse → ถึง flag center
 *  H_UNWIND: (เฉพาะ wrong_side) หมุนกลับ 360° ทิศตรงข้าม → set home
 *  H_DONE  : set mode ตาม physical switch
 */

#ifndef HOMING_H_
#define HOMING_H_

#include <stdint.h>

typedef enum {
    H_IDLE   = 0,
    H_LEAVE  = 1,   /* sensor HIGH ตอน start → ขับออกก่อน */
    H_SEEK   = 2,
    H_COUNT  = 3,
    H_RETURN = 4,
    H_UNWIND = 6,   /* wrong-side → หมุนกลับ 360° ก่อน set home */
    H_DONE   = 5,
} HomingState_t;

void    Homing_Init   (void);
void    Homing_Start  (void);
void    Homing_Update (void);
void    Homing_SetHome(void);   /* zero encoder at current position (Set Home cmd) */
uint8_t Homing_IsDone (void);

#endif /* HOMING_H_ */
