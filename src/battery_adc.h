/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <stdint.h>

/**
 * @brief 배터리 ADC 관리 클래스
 * 
 * 1셀 리튬 배터리 전압을 1:2 분압하여 ADC로 측정합니다.
 * 저전력 동작을 위해 측정 시에만 ADC를 활성화합니다.
 */
class BatteryAdc {
public:
  /**
   * @brief BatteryAdc 싱글톤 인스턴스 반환
   */
  static BatteryAdc &Instance()
  {
    static BatteryAdc sBatteryAdc;
    return sBatteryAdc;
  }

  /**
   * @brief ADC 초기화
   * @return 0: 성공, 음수: 실패
   */
  int Init();

  /**
   * @brief 배터리 전압 측정
   * @return 배터리 전압(mV), 실패 시 음수
   */
  int32_t ReadVoltage();

  /**
   * @brief 배터리 전압으로부터 잔량(%) 계산
   * @param voltage_mv 배터리 전압(mV)
   * @return 배터리 잔량 (0~200, 0.5% 단위, 200 = 100%)
   */
  uint8_t CalculatePercentage(uint32_t voltage_mv);

  /**
   * @brief 마지막으로 측정된 전압 반환
   * @return 전압(mV)
   */
  uint32_t GetLastVoltage() const { return mLastVoltage; }

  /**
   * @brief 마지막으로 계산된 잔량 반환
   * @return 잔량 (0~200, 0.5% 단위)
   */
  uint8_t GetLastPercentage() const { return mLastPercentage; }

  // 배터리 전압 범위
  static constexpr uint32_t BATTERY_VOLTAGE_MAX = 4200; /* 4.2V */
  static constexpr uint32_t BATTERY_VOLTAGE_MIN = 3000; /* 3.0V */

private:
  BatteryAdc() = default;
  ~BatteryAdc() = default;

  // 복사 생성자 및 대입 연산자 삭제 (싱글톤)
  BatteryAdc(const BatteryAdc &) = delete;
  BatteryAdc &operator=(const BatteryAdc &) = delete;

  // ADC 설정 상수
  static constexpr uint8_t VOLTAGE_DIVIDER_RATIO = 2;  // 1:2 분압
  
  // ADC 디바이스 및 설정
  struct adc_dt_spec mAdcChannel;
  struct adc_sequence mSequence;
  int16_t mAdcBuffer;

  // 마지막 측정 값
  uint32_t mLastVoltage = BATTERY_VOLTAGE_MAX;
  uint8_t mLastPercentage = 200; // 100%
  
  bool mInitialized = false;
};