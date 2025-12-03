/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "sht31_sensor.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sht31_sensor, CONFIG_CHIP_APP_LOG_LEVEL);

#define SHT3XD_NODE DT_NODELABEL(sht3xd)

#if DT_NODE_EXISTS(SHT3XD_NODE)
static const struct device *sht31_dev = DEVICE_DT_GET(SHT3XD_NODE);
#else
static const struct device *sht31_dev = NULL;
#endif

static bool sensor_initialized = false;

int sht31_sensor_init(void)
{
#if !DT_NODE_EXISTS(SHT3XD_NODE)
	LOG_ERR("SHT3x sensor not found in device tree");
	return -ENODEV;
#endif

	if (!device_is_ready(sht31_dev)) {
		LOG_ERR("SHT3x device is not ready");
		return -ENODEV;
	}

	sensor_initialized = true;
	LOG_INF("SHT31 sensor initialized successfully");
	return 0;
}

int sht31_sensor_read(int16_t *temperature, uint16_t *humidity)
{
	if (!sensor_initialized || sht31_dev == NULL) {
		LOG_ERR("SHT31 sensor not initialized");
		return -ENODEV;
	}

	struct sensor_value temp_val, hum_val;
	int ret;

	ret = sensor_sample_fetch(sht31_dev);
	if (ret < 0) {
		LOG_ERR("Failed to fetch sensor sample: %d", ret);
		return ret;
	}

	ret = sensor_channel_get(sht31_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_val);
	if (ret < 0) {
		LOG_ERR("Failed to get temperature: %d", ret);
		return ret;
	}

	ret = sensor_channel_get(sht31_dev, SENSOR_CHAN_HUMIDITY, &hum_val);
	if (ret < 0) {
		LOG_ERR("Failed to get humidity: %d", ret);
		return ret;
	}

	/* Convert to Matter format:
	 * Temperature: 100 x temperature in degC (0.01 degC resolution)
	 * Humidity: 100 x humidity in % (0.01% resolution)
	 */
	*temperature = (int16_t)(sensor_value_to_double(&temp_val) * 100);
	*humidity = (uint16_t)(sensor_value_to_double(&hum_val) * 100);

	LOG_DBG("Temperature: %d.%02d C (%d), Humidity: %d.%02d %% (%u)",
		temp_val.val1, temp_val.val2 / 10000,
		*temperature,
		hum_val.val1, hum_val.val2 / 10000,
		*humidity);

	return 0;
}

bool sht31_sensor_is_ready(void)
{
	return sensor_initialized;
}
