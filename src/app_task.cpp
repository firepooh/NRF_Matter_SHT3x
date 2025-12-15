/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_task.h"

#include "app/matter_init.h"
#include "app/task_executor.h"
#include "board/board.h"
#include "clusters/identify.h"
#include "lib/core/CHIPError.h"

#include <app-common/zap-generated/attributes/Accessors.h>

#include <zephyr/logging/log.h>

extern "C" {
#include "sht41_sensor.h"
#include "battery_adc.h"
}

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::DeviceLayer;

namespace
{
constexpr chip::EndpointId kPowerSourceEndpointId = 0;  
constexpr chip::EndpointId kTemperatureSensorEndpointId = 1;
constexpr chip::EndpointId kHumiditySensorEndpointId = 2;

Nrf::Matter::IdentifyCluster sIdentifyCluster(kTemperatureSensorEndpointId);
Nrf::Matter::IdentifyCluster sIdentifyClusterHumidity(kHumiditySensorEndpointId);

#ifdef CONFIG_CHIP_ICD_UAT_SUPPORT
#define UAT_BUTTON_MASK DK_BTN3_MSK
#endif
} /* namespace */

void AppTask::ButtonEventHandler(Nrf::ButtonState state, Nrf::ButtonMask hasChanged)
{
#ifdef CONFIG_CHIP_ICD_UAT_SUPPORT
	if ((UAT_BUTTON_MASK & state & hasChanged)) {
		LOG_INF("ICD UserActiveMode has been triggered.");
		Server::GetInstance().GetICDManager().OnNetworkActivity();
	}
#endif
}

bool AppTask::ReadSensorData(double &temperature, double &humidity)
{
  bool sensor_read_success = false;

  /* 실물 센서 읽기 시도 */
  if (mUseRealSensor && mSensorAvailable) {
    if (sht41_is_ready()) {
      int ret = sht41_read_all(&temperature, &humidity);
      if (ret == 0) {
        sensor_read_success = true;
        LOG_INF("SHT41 sensor read - Temp: %.2f C, Humidity: %.2f %%RH", temperature, humidity);
      } else {
        LOG_WRN("Failed to read SHT41 sensor: %d, using simulated data", ret);
      }
    } else {
      LOG_WRN("SHT41 sensor not ready, using simulated data");
    }
  }

  /* 센서 읽기 실패 시 가상 데이터 생성 */
  if (!mUseRealSensor) {
    /* 가상 온도 데이터 생성 */
    UpdateTemperatureMeasurement();
    
    /* 가상 습도 데이터 생성 */
    UpdateHumidityMeasurement();
    
    /* int16_t를 double로 변환 (100으로 나눔) */
    temperature = (double)GetCurrentTemperature() / 100.0;
    humidity = (double)GetCurrentHumidity() / 100.0;
    
    LOG_INF("Simulated data - Temp: %.2f C, Humidity: %.2f %%RH", temperature, humidity);
  }

  return sensor_read_success;
}

void AppTask::UpdateTemperatureAttribute(bool force_update, int16_t new_temperature)
{
  /* 온도 변화 감지: 소수점 첫째 자리까지 비교 */
  /* 예: 23.15 -> 231, 23.18 -> 231 (같음), 23.25 -> 232 (다름) */
  int16_t temp_rounded_current = (new_temperature + 5) / 10;  /* 반올림하여 소수점 첫째 자리까지 */
  int16_t temp_rounded_previous = (mPreviousTemperature + 5) / 10;

  if (!force_update && temp_rounded_current == temp_rounded_previous) {
    LOG_DBG("Temperature unchanged (%.1f C), skipping update", (double)new_temperature / 100.0);
    return;
  }

  /* 온도 변화 있음 - Matter 속성 업데이트 */
  DeviceLayer::PlatformMgr().ScheduleWork(
    [](intptr_t context) {
      int16_t temp_value = (int16_t)context;
      Protocols::InteractionModel::Status status =
        Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(
          kTemperatureSensorEndpointId, temp_value);

      if (status != Protocols::InteractionModel::Status::Success) {
        LOG_ERR("Updating temperature measurement failed %x", to_underlying(status));
      } else {
        LOG_INF("Temperature updated: %d (%.2f C)", temp_value, (double)temp_value / 100.0);
      }
    },
    (intptr_t)new_temperature
  );

  mPreviousTemperature = new_temperature;
}


void AppTask::UpdateHumidityAttribute(bool force_update, uint16_t new_humidity)
{
  /* 습도 변화 감지: 정수 부분만 비교 */
  /* 예: 65.8% -> 65, 65.2% -> 65 (같음), 66.1% -> 66 (다름) */
  uint16_t humidity_int_current = new_humidity / 100;  /* 정수 부분만 */
  uint16_t humidity_int_previous = mPreviousHumidity / 100;

  if (!force_update && humidity_int_current == humidity_int_previous) {
    LOG_DBG("Humidity unchanged (%d %%RH), skipping update", humidity_int_current);
    return;
  }

  /* 습도 변화 있음 - Matter 속성 업데이트 */
  DeviceLayer::PlatformMgr().ScheduleWork(
    [](intptr_t context) {
      uint16_t humidity_value = (uint16_t)context;
      Protocols::InteractionModel::Status status =
        Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(
          kHumiditySensorEndpointId, humidity_value);

      if (status != Protocols::InteractionModel::Status::Success) {
        LOG_ERR("Updating humidity measurement failed %x", to_underlying(status));
      } else {
        LOG_INF("Humidity updated: %d (%.2f %%RH)", humidity_value, (double)humidity_value / 100.0);
      }
    },
    (intptr_t)new_humidity
  );

  mPreviousHumidity = new_humidity;
}


void AppTask::UpdateBatteryAttributes()
{
  /* Matter 속성 업데이트 */
  DeviceLayer::PlatformMgr().ScheduleWork(
    [](intptr_t context) {
      AppTask *inst = reinterpret_cast<AppTask*>(context);
      
      /* BatVoltage 업데이트 */
      Protocols::InteractionModel::Status status =
        Clusters::PowerSource::Attributes::BatVoltage::Set(
          kPowerSourceEndpointId, inst->GetCurrentBatteryVoltage());

      if (status != Protocols::InteractionModel::Status::Success) {
        LOG_ERR("Updating battery voltage failed %x", to_underlying(status));
      }

      /* BatPercentRemaining 업데이트 */
      status = Clusters::PowerSource::Attributes::BatPercentRemaining::Set(
        kPowerSourceEndpointId, inst->GetCurrentBatteryPercentage());

      if (status != Protocols::InteractionModel::Status::Success) {
        LOG_ERR("Updating battery percentage failed %x", to_underlying(status));
      }
    },
    (intptr_t)this
  );
}


void AppTask::InitializeSHT41Sensor()
{
	constexpr int MAX_RETRIES = 3;
	constexpr int RETRY_DELAY_MS = 50;

	/* SHT41 센서 초기화 시도 (최대 3번 retry) */
	for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
		LOG_INF("SHT41 sensor initialization attempt %d/%d", attempt, MAX_RETRIES);
		
		if ( sht41_sensor_init() == 0) {
			double temp, hum;
			int ret = sht41_read_all(&temp, &hum);
			if (ret == 0) {
				LOG_INF("SHT41 sensor initialized successfully - Temp: %.2f C, Humidity: %.2f %%RH", temp, hum);
				mSensorAvailable = true;

				/* 초기 온습도 값 설정 */
				mCurrentTemperature = (int16_t)(temp * 100.0);
				mCurrentHumidity = (uint16_t)(hum * 100.0);
				mPreviousTemperature = mCurrentTemperature;
				mPreviousHumidity = mCurrentHumidity;
				return;
			} else {
				LOG_WRN("SHT41 sensor read failed: %d (attempt %d/%d)", ret, attempt, MAX_RETRIES);
			}
		} else {
			LOG_WRN("SHT41 sensor not found (attempt %d/%d)", attempt, MAX_RETRIES);
		}

		/* 마지막 시도가 아니면 대기 */
		if (attempt < MAX_RETRIES) {
			k_msleep(RETRY_DELAY_MS);
		}
	}

	/* 모든 시도 실패 - 가상 데이터 사용 */
	LOG_WRN("SHT41 sensor initialization failed after %d attempts, will use simulated data", MAX_RETRIES);
	mSensorAvailable = false;

}


void AppTask::InitializeBatteryAdc()
{
	LOG_INF("Initializing battery ADC...");

	/* Battery ADC 초기화 */
	int ret = BatteryAdc::Instance().Init();
	if (ret == 0) {
		LOG_INF("Battery ADC initialized successfully");
		mBatteryAdcAvailable = true;
		mUseRealBatteryAdc = true;

		/* 초기 배터리 전압 읽기 */
		int32_t voltage_mv = BatteryAdc::Instance().ReadVoltage();
		if (voltage_mv > 0) {
			mCurrentBatteryVoltage = (uint32_t)voltage_mv;
			mCurrentBatteryPercentage = BatteryAdc::Instance().GetLastPercentage();
			LOG_INF("Initial battery voltage: %d mV, percentage: %d (0.5%% units)",
			        mCurrentBatteryVoltage, mCurrentBatteryPercentage);
		} else {
			LOG_WRN("Failed to read initial battery voltage: %d", voltage_mv);
		}
	} else {
		LOG_WRN("Battery ADC initialization failed: %d, will use simulated data", ret);
		mBatteryAdcAvailable = false;
		mUseRealBatteryAdc = false;
	}
}


void AppTask::SensorReadWorkHandler(k_work *work)
{
	if (!work) {
		return;
	}

	/* work에서 AppTask 인스턴스 가져오기 */
	k_work_delayable *delayable_work = k_work_delayable_from_work(work);
	AppTask *instance = CONTAINER_OF(delayable_work, AppTask, mSensorReadWork);

	double temperature = 0.0;
	double humidity = 0.0;

	/* 센서 데이터 읽기 (실물 또는 가상) */
	bool sensor_read_success = instance->ReadSensorData(temperature, humidity);

	if (instance->mUseRealSensor && !sensor_read_success) {
		LOG_WRN("Sensor read failed, will use previous values");
		/* 이전 값 사용 */
		temperature = (double)instance->GetCurrentTemperature() / 100.0;
		humidity = (double)instance->GetCurrentHumidity() / 100.0;
	}

	/* Matter 속성 값으로 변환 (100배) */
	int16_t new_temperature_value = (int16_t)(temperature * 100.0);
	uint16_t new_humidity_value = (uint16_t)(humidity * 100.0);

	bool force_update = (instance->mForceUpdateCount < AppTask::kForceUpdateMaxCount);
	if (force_update) {
		instance->mForceUpdateCount++;
	}

	/* 온도 업데이트 (변화 감지) */
	instance->UpdateTemperatureAttribute(force_update, new_temperature_value);

	/* 습도 업데이트 (변화 감지) */
	instance->UpdateHumidityAttribute(force_update, new_humidity_value);

	/* 다음 1분 후 재스케줄 */
	k_work_schedule(&instance->mSensorReadWork, K_MSEC(kSensorReadIntervalMs));
}


void AppTask::BatteryReadWorkHandler(k_work *work)
{
	if (!work) {
		return;
	}

	/* work에서 AppTask 인스턴스 가져오기 */
	k_work_delayable *delayable_work = k_work_delayable_from_work(work);
	AppTask *instance = CONTAINER_OF(delayable_work, AppTask, mBatteryReadWork);

	/* 실제 배터리 ADC 데이터 사용 시도 */
	if (instance->mUseRealBatteryAdc && instance->mBatteryAdcAvailable) {
		int32_t voltage_mv = BatteryAdc::Instance().ReadVoltage();
		if (voltage_mv > 0) {
			instance->mCurrentBatteryVoltage = (uint32_t)voltage_mv;
			instance->mCurrentBatteryPercentage = BatteryAdc::Instance().GetLastPercentage();
			LOG_INF("Battery ADC read: voltage=%d mV, percentage=%d (0.5%% units)",
			        instance->mCurrentBatteryVoltage,
			        instance->mCurrentBatteryPercentage);
		} else {
			LOG_WRN("Battery ADC read failed: %d, using simulated data", voltage_mv);
		}
	} else {
		/* 가상 데이터 사용 */
		instance->UpdateBatteryVoltage();
		instance->UpdateBatteryPercentage();
		LOG_DBG("Battery simulated: voltage=%d mV, percentage=%d (0.5%% units)",
		        instance->mCurrentBatteryVoltage,
		        instance->mCurrentBatteryPercentage);
	}

	/* 배터리 속성 Matter 업데이트 */
	instance->UpdateBatteryAttributes();

	/* 다음 읽기 스케줄링 */
	if( instance->mForceUpdateCount < AppTask::kForceUpdateMaxCount) {
		/* 부팅후 1분 간격으로 읽기 */
		k_work_schedule(&instance->mBatteryReadWork, K_MSEC(kBatteryReadIntervalShortMs));
	} else {
	  k_work_schedule(&instance->mBatteryReadWork, K_MSEC(kBatteryReadIntervalMs));
	}
}


CHIP_ERROR AppTask::Init()
{
	/* Initialize Matter stack */
	ReturnErrorOnFailure(Nrf::Matter::PrepareServer());

#ifdef CONFIG_DK_LIBRARY	
	if (!Nrf::GetBoard().Init(ButtonEventHandler)) {
		LOG_ERR("User interface initialization failed.");
		return CHIP_ERROR_INCORRECT_STATE;
	}
#endif	

	/* Register Matter event handler that controls the connectivity status LED based on the captured Matter network
	 * state. */
	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(Nrf::Board::DefaultMatterEventHandler, 0));

	ReturnErrorOnFailure(sIdentifyCluster.Init());
	ReturnErrorOnFailure(sIdentifyClusterHumidity.Init());

	/* SHT41 센서 초기화 */
	InitializeSHT41Sensor();

	#if 0
	/* 배터리 ADC 초기화 */
	InitializeBatteryAdc();
	#endif

	return Nrf::Matter::StartServer();
}

#include <zephyr/kernel.h>
#include <nrfx.h>
#include <hal/nrf_nvmc.h>

void configure_regout0_3v0(void)
{
  if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) == 
      (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos)) {
    
    nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_WRITE);
    
    // 3.0V = VOUT 필드를 2 (010)로 설정
    NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~UICR_REGOUT0_VOUT_Msk) |
                        (4UL << UICR_REGOUT0_VOUT_Pos);  // 2 = 3.0V
    
    while (!nrf_nvmc_ready_check(NRF_NVMC));
    
    nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_READONLY);
    
    NVIC_SystemReset();
  }
}

CHIP_ERROR AppTask::StartApp()
{
	/* Regout0을 3.0V로 설정 */
	configure_regout0_3v0();

	ReturnErrorOnFailure(Init());

	/************************************************************************************************************ */
	DataModel::Nullable<int16_t> val;
	Protocols::InteractionModel::Status status =
		Clusters::TemperatureMeasurement::Attributes::MinMeasuredValue::Get(kTemperatureSensorEndpointId, val);

	if (status != Protocols::InteractionModel::Status::Success || val.IsNull()) {
		LOG_ERR("Failed to get temperature measurement min value %x", to_underlying(status));
		return CHIP_ERROR_INCORRECT_STATE;
	}

	mTemperatureSensorMinValue = val.Value();

	status = Clusters::TemperatureMeasurement::Attributes::MaxMeasuredValue::Get(kTemperatureSensorEndpointId, val);

	if (status != Protocols::InteractionModel::Status::Success || val.IsNull()) {
		LOG_ERR("Failed to get temperature measurement max value %x", to_underlying(status));
		return CHIP_ERROR_INCORRECT_STATE;
	}

	mTemperatureSensorMaxValue = val.Value();
	/************************************************************************************************************ */
	// 습도 센서 초기화 추가
  DataModel::Nullable<uint16_t> humidityVal;
  status = Clusters::RelativeHumidityMeasurement::Attributes::MinMeasuredValue::Get(kHumiditySensorEndpointId, humidityVal);

  if (status != Protocols::InteractionModel::Status::Success || humidityVal.IsNull()) {
    LOG_ERR("Failed to get humidity measurement min value %x", to_underlying(status));
    return CHIP_ERROR_INCORRECT_STATE;
  }

  mHumiditySensorMinValue = humidityVal.Value();

  status = Clusters::RelativeHumidityMeasurement::Attributes::MaxMeasuredValue::Get(kHumiditySensorEndpointId, humidityVal);

  if (status != Protocols::InteractionModel::Status::Success || humidityVal.IsNull()) {
    LOG_ERR("Failed to get humidity measurement max value %x", to_underlying(status));
    return CHIP_ERROR_INCORRECT_STATE;
  }

  mHumiditySensorMaxValue = humidityVal.Value();
  /************************************************************************************************************ */


	/* 센서 읽기 work 초기화 및 시작 */
	k_work_init_delayable(&mSensorReadWork, SensorReadWorkHandler);
	/* 초기 센서 읽기는 5초 후 시작 (Matter 초기화 대기) */
	k_work_schedule(&mSensorReadWork, K_MSEC(5000));
	LOG_INF("Sensor reading task started (60 second interval)");

  #if 0
	/* 배터리 읽기 work 초기화 및 시작 */
	k_work_init_delayable(&mBatteryReadWork, BatteryReadWorkHandler);
	/* 초기 배터리 읽기는 10초 후 시작 (Matter 초기화 대기) */
	k_work_schedule(&mBatteryReadWork, K_MSEC(10000));
	LOG_INF("Battery reading task started (600 second interval)");
  #endif

	while (true) {
		Nrf::DispatchNextTask();
	}

	return CHIP_NO_ERROR;
}
