/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the SHT31 sensor
 * @return 0 on success, negative error code on failure
 */
int sht31_sensor_init(void);

/**
 * @brief Read temperature and humidity from SHT31 sensor
 * @param temperature Pointer to store temperature value (in 0.01 degC units)
 * @param humidity Pointer to store humidity value (in 0.01% units)
 * @return 0 on success, negative error code on failure
 */
int sht31_sensor_read(int16_t *temperature, uint16_t *humidity);

/**
 * @brief Check if sensor is ready
 * @return true if sensor is initialized and ready, false otherwise
 */
bool sht31_sensor_is_ready(void);

#ifdef __cplusplus
}
#endif
