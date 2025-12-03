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
  /************************************************************************************************ */


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
  /************************************************************************************************ */


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
  /************************************************************************************************ */


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
  /************************************************************************************************ */


private:
	CHIP_ERROR Init();

  /* 1분 주기 센서 읽기 work */
  k_work_delayable mSensorReadWork;

  static constexpr uint32_t kSensorReadIntervalMs = 60000; /* 60 seconds */
  static constexpr uint16_t kTemperatureMeasurementStep = 100; /* 1 degree Celsius */
  static constexpr uint16_t kHumidityMeasurementStep = 100; /* 1 percent */

  /* 센서 읽기 및 업데이트 work handler */
  static void SensorReadWorkHandler(k_work *work);
  /* SHT31 센서 초기화 */
  void InitializeSHT31Sensor();
  /* 센서 데이터 읽기 (실물 또는 가상) */
  bool ReadSensorData(double &temperature, double &humidity);
  /* 온도 업데이트 (변화 감지 후 Matter 속성 업데이트) */
  void UpdateTemperatureAttribute(int16_t new_temperature);
  /* 습도 업데이트 (변화 감지 후 Matter 속성 업데이트) */
  void UpdateHumidityAttribute(uint16_t new_humidity);
    /* 배터리 상태 업데이트 (가상 데이터만) */
  void UpdateBatteryAttributes();

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

  /* 이전 센서 값 저장 (변화 감지용) */
  int16_t mPreviousTemperature = 0;
  uint16_t mPreviousHumidity = 0;
  
  /* 센서 사용 가능 여부 */
  bool mSensorAvailable = false;
  
  /* 실제 센서 데이터 사용 여부 (true: 실제 센서, false: 가상 데이터) */
  bool mUseRealSensor = true;
};
