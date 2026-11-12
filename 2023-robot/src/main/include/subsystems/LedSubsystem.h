// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

enum LEDPattern {
  SOLID,
  FLASH,
  BREATH,
  WAVE,
  PONG,
  LOADING,
};

class LedSubsystem : public frc2::SubsystemBase {
 public:
  LedSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Write the entire colour buffer to a single colour based on RGB values.
   *
   * @param r The red colour value.
   * @param g The green colour value.
   * @param b The blue colour value.
   */
  void SetRGB(int r, int g, int b);

  /**
   * Write the left side of the robot's leds a single colour based on RGB values.
   *
   * @param r The red colour value.
   * @param g The green colour value.
   * @param b The blue colour value.
   */
  void SetRGBLeft(int r, int g, int b);

  /**
   * Write the right side of the robot's leds a single colour based on RGB values.
   *
   * @param r The red colour value.
   * @param g The green colour value.
   * @param b The blue colour value.
   */
  void SetRGBRight(int r, int g, int b);

  /**
   * Sets which pattern to use to display the colour buffer.
   *
   * @param pattern Which LEDPattern to run on the LED's.
   * @param period The period over which to run the pattern.
   */
  void SetPattern(LEDPattern pattern, units::millisecond_t period);

  /**
   * Stores the pattern & colours to be recalled at a later time.
   */
  void StorePattern();

  /**
   * Restores the previously stored pattern & colours.
   */
  void RestorePattern();

  /**
   * Set's the front LED's on or off depending on the isOn value.
   */
  void SetFrontLEDsOn(bool isOn) {
    m_isFrontLEDsOff = isOn;
  }

 private:
  frc::AddressableLED m_led { LEDConstants::kpwmPort };
  std::array<frc::AddressableLED::LEDData, LEDConstants::kLength> m_ledColourBuffer;
  std::array<frc::AddressableLED::LEDData, LEDConstants::kLength> m_ledPatternBuffer;
  std::array<frc::AddressableLED::LEDData, LEDConstants::kLength> m_outputBuffer;
  
  LEDPattern m_pattern;
  units::millisecond_t m_period;
  units::millisecond_t m_periodStart;
  bool m_isNewPattern;
  bool m_isFrontLEDsOff;

  LEDPattern m_patternStored;
  units::millisecond_t m_periodStored;
  std::array<frc::AddressableLED::LEDData, LEDConstants::kLength> m_ledColourBufferStored;

  std::string_view TranslatePattern(LEDPattern pattern) {
    switch (pattern) {
      case SOLID:
        return "SOLID";
      case FLASH:
        return "FLASH";
      case BREATH:
        return "BREATH";
      case WAVE:
        return "WAVE";
      case PONG:
        return "PONG";
      case LOADING:
        return "LOADING";
    }
    return "Undefined";
  }
};