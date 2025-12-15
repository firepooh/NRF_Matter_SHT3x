/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "sht41_sensor.h"

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/sht4x.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sht41_sensor, LOG_LEVEL_INF);

/* SHT41 디바이스 노드 */
static const struct device *sht41_dev = DEVICE_DT_GET_ANY(sensirion_sht4x);
bool sht41_ready = false;

int sht41_sensor_init(void)
{
  if (!sht41_dev) {
    LOG_ERR("Failed to get SHT41 device");
    return -ENODEV;
  }

  if (!device_is_ready(sht41_dev)) {
    LOG_ERR("SHT41 device is not ready");
    return -ENODEV;
  }

  LOG_INF("SHT41 sensor initialized successfully");

  sht41_ready = true;

  return 0;
}


bool sht41_is_ready(void)
{
  if (!sht41_dev || !sht41_ready) {
    return false;
  }

  return true;
}


int sht41_read_temperature(double *temperature)
{
  struct sensor_value temp_value;
  int ret;

  if (!sht41_is_ready()) {
    LOG_ERR("SHT41 device is not ready");
    return -ENODEV;
  }

  /* 센서 데이터 샘플링 */
  ret = sensor_sample_fetch(sht41_dev);
  if (ret) {
    LOG_ERR("Failed to fetch sensor data: %d", ret);
    return ret;
  }

  /* 온도 값 읽기 */
  ret = sensor_channel_get(sht41_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
  if (ret) {
    LOG_ERR("Failed to get temperature: %d", ret);
    return ret;
  }

  /* sensor_value를 double로 변환 */
  *temperature = sensor_value_to_double(&temp_value);

  LOG_DBG("Temperature: %.2f °C", *temperature);
  return 0;
}

int sht41_read_humidity(double *humidity)
{
  struct sensor_value hum_value;
  int ret;

  if (!sht41_is_ready()) {
    LOG_ERR("SHT41 device is not ready");
    return -ENODEV;
  }

  /* 센서 데이터 샘플링 */
  ret = sensor_sample_fetch(sht41_dev);
  if (ret) {
    LOG_ERR("Failed to fetch sensor data: %d", ret);
    return ret;
  }

  /* 습도 값 읽기 */
  ret = sensor_channel_get(sht41_dev, SENSOR_CHAN_HUMIDITY, &hum_value);
  if (ret) {
    LOG_ERR("Failed to get humidity: %d", ret);
    return ret;
  }

  /* sensor_value를 double로 변환 */
  *humidity = sensor_value_to_double(&hum_value);

  LOG_DBG("Humidity: %.2f %%RH", *humidity);
  return 0;
}

int sht41_read_all(double *temperature, double *humidity)
{
  struct sensor_value temp_value, hum_value;
  int ret;

  if (!sht41_is_ready()) {
    LOG_ERR("SHT41 device is not ready");
    return -ENODEV;
  }

  /* 센서 데이터 샘플링 (한 번만 호출) */
  ret = sensor_sample_fetch(sht41_dev);
  if (ret) {
    LOG_ERR("Failed to fetch sensor data: %d", ret);
    return ret;
  }

  /* 온도 값 읽기 */
  ret = sensor_channel_get(sht41_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
  if (ret) {
    LOG_ERR("Failed to get temperature: %d", ret);
    return ret;
  }
  *temperature = sensor_value_to_double(&temp_value);

  /* 습도 값 읽기 */
  ret = sensor_channel_get(sht41_dev, SENSOR_CHAN_HUMIDITY, &hum_value);
  if (ret) {
    LOG_ERR("Failed to get humidity: %d", ret);
    return ret;
  }
  *humidity = sensor_value_to_double(&hum_value);

  LOG_INF("Temp: %.2f °C, Humidity: %.2f %%RH", *temperature, *humidity);
  return 0;
}
