// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LedSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/Timer.h>

LedSubsystem::LedSubsystem() {
  m_isNewPattern = false;
  m_isFrontLEDsOff = false;

  m_led.SetLength(LEDConstants::kLength);
  LedSubsystem::SetRGB(255, 255, 255);
  LedSubsystem::SetPattern(LEDPattern::SOLID, 0_ms);
  m_led.Start();
};

// This method will be called once per scheduler run
void LedSubsystem::Periodic() {
  // If there is a new pattern since last cycle, copy the colour buffer to the pattern buffer.
  if (m_isNewPattern) {
    m_isNewPattern = false;
    m_periodStart = frc::Timer::GetFPGATimestamp();
    if (m_pattern == LEDPattern::LOADING || m_pattern == LEDPattern::PONG) {
      for (int i = 0; i < LEDConstants::kLength; i++) {
        m_ledPatternBuffer[i].r = 0;
        m_ledPatternBuffer[i].g = 0;
        m_ledPatternBuffer[i].b = 0;
      }
    } else {
      memcpy(m_ledPatternBuffer.data(), m_ledColourBuffer.data(), m_ledColourBuffer.size() * 4);
    }
  }

  double ledBrightness = 0;
  // Perform the next animation step on the pattern buffer.
  switch (m_pattern) {
    case LEDPattern::SOLID:
      break;
    case LEDPattern::FLASH:
      if (frc::Timer::GetFPGATimestamp() > m_periodStart + (m_period / 2)) {
        for (int i = 0; i < LEDConstants::kLength; i++) {
          m_ledPatternBuffer[i].SetRGB(0, 0, 0);
        }
        if (frc::Timer::GetFPGATimestamp() > m_periodStart + m_period) {
          for (int i = 0; i < LEDConstants::kLength; i++) {
            m_ledPatternBuffer[i] = m_ledColourBuffer[i];
          }
          m_periodStart = frc::Timer::GetFPGATimestamp();
        }
      }
      break;
    case LEDPattern::BREATH:
      ledBrightness = ((frc::Timer::GetFPGATimestamp() - m_periodStart) / m_period).value() * 2;
      if (ledBrightness > 1.0) {
        ledBrightness = 1.0 - (ledBrightness - 1.0);
      }

      if (ledBrightness < 0) {
        ledBrightness = 0;
      }

      // frc::DataLogManager::Log(fmt::format("ledBrightness: {}", ledBrightness));

      for (int i = 0; i < LEDConstants::kLength; i++) {
        m_ledPatternBuffer[i].SetRGB(
          m_ledColourBuffer[i].r * ledBrightness,
          m_ledColourBuffer[i].g * ledBrightness,
          m_ledColourBuffer[i].b * ledBrightness);
      }

      if (frc::Timer::GetFPGATimestamp() > m_periodStart + m_period) {
        m_periodStart = frc::Timer::GetFPGATimestamp();
      }
      break;
    case LEDPattern::WAVE:
      if (frc::Timer::GetFPGATimestamp() > m_periodStart + m_period) {
        m_periodStart = frc::Timer::GetFPGATimestamp();
      }
      break;
    case LEDPattern::PONG:
      if (frc::Timer::GetFPGATimestamp() > m_periodStart + m_period) {
        m_periodStart = frc::Timer::GetFPGATimestamp();
      }
      break;
    case LEDPattern::LOADING:
      double columnSize = LEDConstants::kLength / 4.0;
      double percentDone = ((frc::Timer::GetFPGATimestamp() - m_periodStart) / m_period).value();
      int ledsToTurnOn = columnSize * percentDone;

      for (int i = 0; i < ledsToTurnOn; i++) {
        m_ledPatternBuffer[i] = m_ledColourBuffer[i];
        m_ledPatternBuffer[columnSize + columnSize - i] =
          m_ledColourBuffer[columnSize + columnSize - i];
        m_ledPatternBuffer[i + columnSize + columnSize] =
          m_ledColourBuffer[i + columnSize + columnSize];
        m_ledPatternBuffer[columnSize + columnSize + columnSize + columnSize - i] =
          m_ledColourBuffer[columnSize + columnSize + columnSize + columnSize - i];
      }

      if (frc::Timer::GetFPGATimestamp() > m_periodStart + m_period) {
        m_periodStart = frc::Timer::GetFPGATimestamp();
      }
      break;
  }

  memcpy(m_outputBuffer.data(), m_ledPatternBuffer.data(), m_ledPatternBuffer.size() * 4);

  if (m_isFrontLEDsOff) {
    for (int i = 0; i < (LEDConstants::kLength / 4) + 1; i++) {
      m_outputBuffer[i].r = 0;
      m_outputBuffer[i].g = 0;
      m_outputBuffer[i].b = 0;
    }
    for (int i = (LEDConstants::kLength / 4) * 3; i < LEDConstants::kLength; i++) {
      m_outputBuffer[i].r = 0;
      m_outputBuffer[i].g = 0;
      m_outputBuffer[i].b = 0;
    }
  } else {
    for (int i = 0; i < (LEDConstants::kLength / 4) + 1; i++) {
      m_outputBuffer[i] = m_ledPatternBuffer[i];
    }
    for (int i = (LEDConstants::kLength / 4) * 3; i < LEDConstants::kLength; i++) {
      m_outputBuffer[i] = m_ledPatternBuffer[i];
    }
  }

  // Write the pattern buffer to the LED.
  m_led.SetData(m_outputBuffer);
}

void LedSubsystem::SetRGB(int r, int g, int b) {
  for (int i = 0; i < LEDConstants::kLength; i++) {
    m_ledColourBuffer[i].SetRGB(r, g, b);
  }
  frc::DataLogManager::Log(fmt::format("Set LEDs to RGB values: ({}, {}, {})", r, g, b));
}

// TODO: Check that this is actually the left and right side. Might need to mess with the length
// it's iterating over a bit.
void LedSubsystem::SetRGBLeft(int r, int g, int b) {
  for (int i = 0; i < LEDConstants::kLength / 2; i++) {
    m_ledColourBuffer[i].SetRGB(r, g, b);
  }
  frc::DataLogManager::Log(fmt::format("Set LEDs to RGB values: ({}, {}, {})", r, g, b));
}

// TODO: Check that this is actually the left and right side. Might need to mess with the length
// it's iterating over a bit.
void LedSubsystem::SetRGBRight(int r, int g, int b) {
  for (int i = LEDConstants::kLength / 2; i < LEDConstants::kLength; i++) {
    m_ledColourBuffer[i].SetRGB(r, g, b);
  }
  frc::DataLogManager::Log(fmt::format("Set LEDs to RGB values: ({}, {}, {})", r, g, b));
}

void LedSubsystem::SetPattern(LEDPattern pattern, units::millisecond_t period) {
  m_pattern = pattern;
  m_period = period;
  m_isNewPattern = true;
  frc::DataLogManager::Log(
    fmt::format("Set LEDs pattern to: {} with period: {}", TranslatePattern(m_pattern), m_period));
}

void LedSubsystem::StorePattern() {
  memcpy(&m_ledColourBufferStored, &m_ledColourBuffer, m_ledColourBuffer.size() * 4);
  m_patternStored = m_pattern;
  m_periodStored = m_period;
  frc::DataLogManager::Log("Stored LED colours, pattern and period.");
}

void LedSubsystem::RestorePattern() {
  memcpy(&m_ledColourBuffer, &m_ledColourBufferStored, m_ledColourBuffer.size() * 4);
  m_pattern = m_patternStored;
  m_isNewPattern = true;
  m_period = m_periodStored;
  frc::DataLogManager::Log("Restored LED colours, pattern and period.");
}