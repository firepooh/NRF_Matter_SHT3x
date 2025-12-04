/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "battery_adc.h"

#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(battery_adc, CONFIG_CHIP_APP_LOG_LEVEL);

int BatteryAdc::Init()
{
	/* Initialize ADC channel from device tree */
	mAdcChannel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
	if (!mAdcChannel.dev) {
		LOG_ERR("ADC device not found in device tree");
		return -ENODEV;
	}

	/* STEP 4.1 - Define ADC sequence configuration */
	mSequence = {
		.buffer = &mAdcBuffer,
		.buffer_size = sizeof(mAdcBuffer),
	};

	/* STEP 3.3 - Validate that the ADC peripheral (SAADC) is ready */
	if (!adc_is_ready_dt(&mAdcChannel)) {
		LOG_ERR("ADC controller device %s not ready", mAdcChannel.dev->name);
		return -ENODEV;
	}

	/* STEP 3.4 - Setup the ADC channel */
	if (adc_channel_setup_dt(&mAdcChannel) < 0) {
		LOG_ERR("Could not setup ADC channel");
		return -EIO;
	}

	/* STEP 4.2 - Initialize the ADC sequence */
	if (adc_sequence_init_dt(&mAdcChannel, &mSequence) < 0) {
		LOG_ERR("Could not initialize ADC sequence");
		return -EIO;
	}

	mInitialized = true;

	/* Perform initial ADC read to verify functionality */
	int32_t val_mv = ReadVoltage();
	if (val_mv < 0) {
		LOG_ERR("Initial ADC read failed");
		mInitialized = false;
		return -EIO;
	}

	LOG_INF("Battery ADC initialized: %d mV", val_mv);
	return 0;
}


int32_t BatteryAdc::ReadVoltage()
{
	if (!mInitialized) {
		LOG_ERR("Battery ADC not initialized");
		return -ENODEV;
	}

	/* Perform ADC read */
	int err = adc_read(mAdcChannel.dev, &mSequence);
	if (err < 0) {
		LOG_ERR("ADC read failed: %d", err);
		return err;
	}

	/* Convert raw value to millivolts */
	int32_t val_mv = (int32_t)mAdcBuffer;
	err = adc_raw_to_millivolts_dt(&mAdcChannel, &val_mv);
	if (err < 0) {
		LOG_ERR("ADC raw to mV conversion failed: %d", err);
		return err;
	}

	/* Apply voltage divider ratio (1:2 divider) to get actual battery voltage */
	val_mv = val_mv * VOLTAGE_DIVIDER_RATIO;

	/* Store the last measured voltage */
	mLastVoltage = (uint32_t)val_mv;

	/* Calculate and store battery percentage */
	mLastPercentage = CalculatePercentage(mLastVoltage);

	LOG_DBG("Battery voltage: %d mV, percentage: %d (0.5%% unit)",
	        mLastVoltage, mLastPercentage);

	return (int32_t)mLastVoltage;
}


uint8_t BatteryAdc::CalculatePercentage(uint32_t voltage_mv)
{
	/* Calculate battery percentage based on voltage
	   3000mV = 0%, 4200mV = 100% (Matter: 0~200, 0.5% unit) */

	if (voltage_mv <= BATTERY_VOLTAGE_MIN) {
		return 0;
	} else if (voltage_mv >= BATTERY_VOLTAGE_MAX) {
		return 200;
	}

	uint32_t range = BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN;
	uint32_t current = voltage_mv - BATTERY_VOLTAGE_MIN;
	uint8_t percentage = (current * 200) / range;

	return percentage;
}

