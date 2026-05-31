/*
 * encoder.c
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */


#include "encoder.h"

void Encoder_Init(Encoder_t *enc, TIM_HandleTypeDef *htim) {
    enc->htim = htim;
    enc->last_counter = 0;
    enc->position_rad = 0;
    enc ->position_raw = 0;
    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
}

void Encoder_Update(Encoder_t *enc) {

	uint32_t current_counter = __HAL_TIM_GET_COUNTER(enc->htim);

	int32_t diff = (int32_t)(current_counter - enc->last_counter);

	/* ── Wrap correction ──────────────────────────────────────────────────
	 *  delta method ของ uint32 ถูกต้องเองเฉพาะเมื่อ ARR = 0xFFFFFFFF (วนที่ 2^32)
	 *  ถ้า ARR เล็กกว่า (เช่น 0xFFFF หรือ CPR-1) counter จะวนเร็ว → diff กระโดด
	 *  → อ่าน ARR มาแก้: ถ้า |diff| > ครึ่ง period แสดงว่าเพิ่งวน → ชดเชย
	 *  (Encoder_Update เรียกทุก 1ms → ขยับ < ครึ่ง period ต่อ tick เสมอ)         */
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(enc->htim);
	if (arr != 0xFFFFFFFFu) {
		int32_t period = (int32_t)arr + 1;
		if      (diff >  (period >> 1)) diff -= period;
		else if (diff < -(period >> 1)) diff += period;
	}

	enc->position_raw += diff;
	enc->last_counter = current_counter;

	enc->position_rad = (float)enc->position_raw * (2.0f * PI / ENCODER_CPR);
	enc->position_deg = enc->position_rad * (180.0f / PI);
}

// ---- Getters ---
int32_t Encoder_GetPositionRaw(Encoder_t *enc) {
    return enc->position_raw;
}

float Encoder_GetPositionRad(Encoder_t *enc) {
    return enc->position_rad;
}

float Encoder_GetPositionDeg(Encoder_t *enc) {
    return enc->position_deg;
}
