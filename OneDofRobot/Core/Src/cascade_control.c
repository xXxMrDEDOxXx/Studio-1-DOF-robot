/*
 * cascade_control.c
 *
 *  Created on: 29 เม.ย. 2569
 *      Author: POND
 */


#include "cascade_control.h"
#include "encoder.h"
#include "trajectory.h"
#include <math.h>

// ---------------- ดึงค่าฮาร์ดแวร์จาก main.c ----------------
extern TIM_HandleTypeDef htim1; // PWM PA8
extern TIM_HandleTypeDef htim2; // QEI
extern Encoder_t henc2;

// ลบคำว่า extern ออก แล้วกำหนดค่าเริ่มต้นได้ตามปกติเลยครับ
float monitor_V_in = 0.0f;

// ---------------- Private Variables (ตัวแปรภายใน) ----------------
float q_out = 0.0f;       // ตำแหน่งจริง
static float prev_q_out = 0.0f;
static float qd_out = 0.0f;      // ความเร็วจริง


// ---------------- Public Variables (กำหนดค่าเริ่มต้น) ----------------
float target_q = 0.0f;
float target_qd = 0.0f;
float qdd_out = 0.0f;
float j_out = 0.0f;

// ---------------- Setup PID Controllers ----------------
PID_Controller pos_ctrl = { .Kp = 20.0f, .Ki = 0.0f, .Kd = 0.0f, .integral = 0.0f, .prev_error = 0.0f, .integral_limit = 50.0f };
PID_Controller vel_ctrl = { .Kp = 4.0f,  .Ki = 0.0f, .Kd = 0.0f, .integral = 0.0f, .prev_error = 0.0f, .integral_limit = MAX_VOLTAGE };
static float K_ff = 0.1045f; //ke

// ---------------- Private Functions (ฟังก์ชันซ่อนภายใน) ----------------
// ใส่ static เพื่อไม่ให้ไฟล์อื่นเรียกใช้ได้โดยตรง
static float calculate_pid(PID_Controller *pid, float error, float dt) {
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}

static void Motor_Drive(float V_in) {



    if (V_in >= 0.0f) {
        HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(Motor_Dir_GPIO_Port, Motor_Dir_Pin, GPIO_PIN_SET);
        V_in = -V_in;
    }

    if (V_in > MAX_VOLTAGE) V_in = MAX_VOLTAGE;
    monitor_V_in = V_in;
    uint32_t duty = (uint32_t)((V_in / MAX_VOLTAGE) * PWM_ARR_MAX);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
}

// ---------------- Public Functions ----------------
// ฟังก์ชันล้างค่า เผื่อใช้สำหรับรีเซ็ตระบบ
void Cascade_Control_Init(void) {
    q_out = 0.0f;
    prev_q_out = 0.0f;
    qd_out = 0.0f;
    target_q = 0.0f;
    target_qd = 0.0f;
}

// ลูปคำนวณหลัก ที่จะถูกเรียกใน Timer Interrupt
void Cascade_Control_Update(float ref_q, float ref_qd) {

    // สร้างตัวแปรนับรอบ และตัวแปรเก็บค่าเป้าหมายความเร็ว (ต้องมี static)
    static uint8_t pos_loop_counter = 0;
    static float current_q_dot_ref = 0.0f; // เก็บค่าที่คำนวณจาก Position Loop ไว้ใช้ต่อ

    static float prev_qd_out = 0.0f;
    static float prev_qdd_out = 0.0f;

    // อ่านค่า Encoder และคำนวณความเร็ว (ทำทุก 1 ms)
    Encoder_Update(&henc2);
    q_out = Encoder_GetPositionRad(&henc2);

    float raw_qd = (q_out - prev_q_out) / DT_VEL; // <--- ใช้ DT_VEL
    float alpha = 0.05f;
    qd_out = (alpha * raw_qd) + ((1.0f - alpha) * qd_out);
    prev_q_out = q_out;

    float raw_qdd = (qd_out - prev_qd_out) / DT_VEL; 	//  qdd_out
	float alpha_a = 0.02f; // ต้องกรองหนักกว่าความเร็ว (ค่าน้อย = กรองเยอะ)
	qdd_out = (alpha_a * raw_qdd) + ((1.0f - alpha_a) * qdd_out);
	prev_qd_out = qd_out;

	float raw_j = (qdd_out - prev_qdd_out) / DT_VEL;	// j_out
	float alpha_j = 0.005f; // ต้องกรองหนักที่สุด! (เพราะ Diff รอบที่ 3 Noise จะเยอะมาก)
	j_out = (alpha_j * raw_j) + ((1.0f - alpha_j) * j_out);
	prev_qdd_out = qdd_out;

    // ==========================================
    // 🟢 OUTER LOOP: Position (รันทุกๆ 10 ms)
    // ==========================================
    pos_loop_counter++;
    if (pos_loop_counter >= 10) {

        float q_error = ref_q - q_out;

        // จุดสังเกตสำคัญ: ต้องส่งค่า DT_POS เข้าไปใน calculate_pid
        current_q_dot_ref = calculate_pid(&pos_ctrl, q_error, DT_POS) + ref_qd;

        pos_loop_counter = 0; // รีเซ็ตตัวนับกลับไป 0
    }

    // ==========================================
    // 🔴 INNER LOOP: Velocity (รันทุกๆ 1 ms)
    // ==========================================
    // เอา current_q_dot_ref (ที่อัปเดตทุก 10ms) มาลบกับ qd_out (ที่อัปเดตทุก 1ms)
    float qd_error = current_q_dot_ref - qd_out;

    // จุดสังเกตสำคัญ: ต้องส่งค่า DT_VEL เข้าไปใน calculate_pid
    float V_VEL = calculate_pid(&vel_ctrl, qd_error, DT_VEL);

    //  Feed Forward & Drive (รันทุก 1 ms)
    float motor_speed_ref = ref_qd * GEAR_RATIO;
    float V_FF = K_ff * motor_speed_ref;

    Motor_Drive(V_VEL + V_FF);
}

