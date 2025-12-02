/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include "board/board.h"

#include <platform/CHIPDeviceLayer.h>

struct Identify;

class AppTask {
public:
	static AppTask &Instance()
	{
		static AppTask sAppTask;
		return sAppTask;
	};

	CHIP_ERROR StartApp();

	/* Defined by cluster temperature measured value = 100 x temperature in degC with resolution of
	 * 0.01 degC. */
	void UpdateTemperatureMeasurement()
	{
		/* Linear temperature increase that is wrapped around to min value after reaching the max value. */
		if (mCurrentTemperature < mTemperatureSensorMaxValue) {
			mCurrentTemperature += kTemperatureMeasurementStep;
		} else {
			mCurrentTemperature = mTemperatureSensorMinValue;
		}
	}

	int16_t GetCurrentTemperature() const { return mCurrentTemperature; }

  /* Defined by cluster humidity measured value = 100 x humidity in % with resolution of 0.01 %. */
  void UpdateHumidityMeasurement()
  {
    /* Linear humidity increase that is wrapped around to min value after reaching the max value. */
    if (mCurrentHumidity < mHumiditySensorMaxValue) {
      mCurrentHumidity += kHumidityMeasurementStep;
    } else {
      mCurrentHumidity = mHumiditySensorMinValue;
    }
  }

  uint16_t GetCurrentHumidity() const { return mCurrentHumidity; }

  /* Battery voltage in mV (3000-4200mV for Li-ion) */
  void UpdateBatteryVoltage()
  {
    /* 배터리 전압 시뮬레이션: 4200mV에서 3000mV까지 감소 */
    if (mCurrentBatteryVoltage > mBatteryVoltageMin) {
      mCurrentBatteryVoltage -= kBatteryVoltageStep;
    } else {
      mCurrentBatteryVoltage = mBatteryVoltageMax;
    }
  }

  uint32_t GetCurrentBatteryVoltage() const { return mCurrentBatteryVoltage; }

  /* Battery percentage 0-200 (0.5% units, 200 = 100%) */
  void UpdateBatteryPercentage()
  {
    /* 배터리 잔량 시뮬레이션: 전압 기반으로 계산 */
    /* 3000mV = 0%, 4200mV = 100% (200 in 0.5% units) */
    uint32_t range = mBatteryVoltageMax - mBatteryVoltageMin;
    uint32_t current = mCurrentBatteryVoltage - mBatteryVoltageMin;
    mCurrentBatteryPercentage = (current * 200) / range;
  }

  uint8_t GetCurrentBatteryPercentage() const { return mCurrentBatteryPercentage; }

private:
	CHIP_ERROR Init();
	k_timer mTimer;

	static constexpr uint32_t kTemperatureMeasurementIntervalMs = 60000; /* 60 seconds */
	static constexpr uint16_t kTemperatureMeasurementStep = 100; /* 1 degree Celsius */

  static constexpr uint16_t kHumidityMeasurementStep = 100; /* 1 percent */

	static void UpdateTemperatureTimeoutCallback(k_timer *timer);

	static void ButtonEventHandler(Nrf::ButtonState state, Nrf::ButtonMask hasChanged);

	int16_t mTemperatureSensorMaxValue = 0;
	int16_t mTemperatureSensorMinValue = 0;
	int16_t mCurrentTemperature = 0;

	// 습도 센서 관련 멤버 변수 추가
	uint16_t mHumiditySensorMaxValue = 0;
	uint16_t mHumiditySensorMinValue = 0;
	uint16_t mCurrentHumidity = 0;

	// 배터리 관련 멤버 변수 추가
  static constexpr uint32_t mBatteryVoltageMax = 4200; /* 4.2V in mV */
  static constexpr uint32_t mBatteryVoltageMin = 3000; /* 3.0V in mV */
  static constexpr uint32_t kBatteryVoltageStep = 50;  /* 50mV 감소 */
  
  uint32_t mCurrentBatteryVoltage = mBatteryVoltageMax;
  uint8_t mCurrentBatteryPercentage = 200; /* 100% = 200 in 0.5% units */	
};
