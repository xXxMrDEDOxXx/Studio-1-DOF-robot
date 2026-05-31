/*
 * velocity_tune.h  —  Deprecated wrapper
 * ─────────────────────────────────────────────────────────────────────────────
 * ไฟล์นี้ยังคงไว้เพื่อ backward compatibility กับ main.c เก่า
 * โค้ดทั้งหมดย้ายไป dashboard.h / dashboard.c แล้ว
 *
 * แนะนำให้เปลี่ยน:
 *   #include "velocity_tune.h"    →  #include "dashboard.h"
 *   VelocityTune_Init()           →  Dashboard_Init()
 *   VelocityTune_Update()         →  Dashboard_Update()
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef VELOCITY_TUNE_H_
#define VELOCITY_TUNE_H_

#include "dashboard.h"

/* Backward-compatibility aliases */
#define VelocityTune_Init    Dashboard_Init
#define VelocityTune_Update  Dashboard_Update

#endif /* VELOCITY_TUNE_H_ */
