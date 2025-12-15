/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SHT41 센서 초기화
 * 
 * @return 0 성공, 음수 에러 코드
 */
int sht41_sensor_init(void);

/**
 * @brief SHT41 센서에서 온도 읽기
 * 
 * @param temperature 온도 값 저장 포인터 (섭씨, °C)
 * @return 0 성공, 음수 에러 코드
 */
int sht41_read_temperature(double *temperature);

/**
 * @brief SHT41 센서에서 습도 읽기
 * 
 * @param humidity 습도 값 저장 포인터 (상대습도, %RH)
 * @return 0 성공, 음수 에러 코드
 */
int sht41_read_humidity(double *humidity);

/**
 * @brief SHT41 센서에서 온도와 습도를 동시에 읽기
 * 
 * @param temperature 온도 값 저장 포인터 (섭씨, °C)
 * @param humidity 습도 값 저장 포인터 (상대습도, %RH)
 * @return 0 성공, 음수 에러 코드
 */
int sht41_read_all(double *temperature, double *humidity);
/**
 * @brief SHT31 센서가 준비되었는지 확인
 * 
 * @return true 준비됨, false 준비 안됨
 */
bool sht41_is_ready(void);

#ifdef __cplusplus
}
#endif
