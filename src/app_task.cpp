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

void AppTask::UpdateTemperatureTimeoutCallback(k_timer *timer)
{
  if (!timer || !timer->user_data) {
    return;
  }

  DeviceLayer::PlatformMgr().ScheduleWork(
    [](intptr_t p) {
      // 온도 업데이트
      AppTask::Instance().UpdateTemperatureMeasurement();

      Protocols::InteractionModel::Status status =
        Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Set(
          kTemperatureSensorEndpointId, AppTask::Instance().GetCurrentTemperature());

      if (status != Protocols::InteractionModel::Status::Success) {
        LOG_ERR("Updating temperature measurement failed %x", to_underlying(status));
      }

      // 습도 업데이트
      AppTask::Instance().UpdateHumidityMeasurement();

      status = Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(
        kHumiditySensorEndpointId, AppTask::Instance().GetCurrentHumidity());

      if (status != Protocols::InteractionModel::Status::Success) {
        LOG_ERR("Updating humidity measurement failed %x", to_underlying(status));
      }

      // 배터리 전압 업데이트
      AppTask::Instance().UpdateBatteryVoltage();
      AppTask::Instance().UpdateBatteryPercentage();

      DataModel::Nullable<uint32_t> batteryVoltage(AppTask::Instance().GetCurrentBatteryVoltage());
      status = Clusters::PowerSource::Attributes::BatVoltage::Set(0, batteryVoltage);

      if (status != Protocols::InteractionModel::Status::Success) {
        LOG_ERR("Updating battery voltage failed %x", to_underlying(status));
      } else {
        LOG_INF("Battery voltage updated: %u mV", AppTask::Instance().GetCurrentBatteryVoltage());
      }

      // 배터리 잔량 업데이트
      DataModel::Nullable<uint8_t> batteryPercent(AppTask::Instance().GetCurrentBatteryPercentage());
      status = Clusters::PowerSource::Attributes::BatPercentRemaining::Set(0, batteryPercent);

      if (status != Protocols::InteractionModel::Status::Success) {
        LOG_ERR("Updating battery percentage failed %x", to_underlying(status));
      } else {
        LOG_INF("Battery percentage updated: %u (%u%%)", 
                AppTask::Instance().GetCurrentBatteryPercentage(),
                AppTask::Instance().GetCurrentBatteryPercentage() / 2 );
      }      

    },
    reinterpret_cast<intptr_t>(timer->user_data));
}

#if 0
void AppTask::UpdateHumidityTimeoutCallback(k_timer *timer)
{
  if (!timer || !timer->user_data) {
    return;
  }

  DeviceLayer::PlatformMgr().ScheduleWork(
    [](intptr_t p) {
      AppTask::Instance().UpdateHumidityMeasurement();

      Protocols::InteractionModel::Status status =
        Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Set(
          kHumiditySensorEndpointId, AppTask::Instance().GetCurrentHumidity());

      if (status != Protocols::InteractionModel::Status::Success) {
        LOG_ERR("Updating humidity measurement failed %x", to_underlying(status));
      }
    },
    reinterpret_cast<intptr_t>(timer->user_data));
}
#endif


CHIP_ERROR AppTask::Init()
{
	/* Initialize Matter stack */
	ReturnErrorOnFailure(Nrf::Matter::PrepareServer());

	if (!Nrf::GetBoard().Init(ButtonEventHandler)) {
		LOG_ERR("User interface initialization failed.");
		return CHIP_ERROR_INCORRECT_STATE;
	}

	/* Register Matter event handler that controls the connectivity status LED based on the captured Matter network
	 * state. */
	ReturnErrorOnFailure(Nrf::Matter::RegisterEventHandler(Nrf::Board::DefaultMatterEventHandler, 0));

	ReturnErrorOnFailure(sIdentifyCluster.Init());
	ReturnErrorOnFailure(sIdentifyClusterHumidity.Init());

	return Nrf::Matter::StartServer();
}

CHIP_ERROR AppTask::StartApp()
{
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
#if 1
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
#endif	
  /************************************************************************************************************ */
	k_timer_init(&mTimer, AppTask::UpdateTemperatureTimeoutCallback, nullptr);
	k_timer_user_data_set(&mTimer, this);
	k_timer_start(&mTimer, K_MSEC(kTemperatureMeasurementIntervalMs), K_MSEC(kTemperatureMeasurementIntervalMs));
	/************************************************************************************************************ */





	while (true) {
		Nrf::DispatchNextTask();
	}

	return CHIP_NO_ERROR;
}
