// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/LedSubsystem.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SetLedCommand : public frc2::CommandHelper<frc2::CommandBase, SetLedCommand> {
 public:
  /**
   * @param subsystem The led subsystem.
   * @param r The red value.
   * @param g The green value.
   * @param b The blue value.
   * @param pattern The pattern to use.
   * @param period The period of the pattern.
   * @param timeToDisplay How long to display the colour/pattern before returning to the previous
   * colour/pattern. Will stay as the new pattern/colour if this value is 0.
   */
  SetLedCommand(
    LedSubsystem* subsystem,
    int r,
    int g,
    int b,
    LEDPattern pattern,
    units::millisecond_t period,
    units::millisecond_t timeToDisplay = units::millisecond_t { 0 });

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  LedSubsystem* m_ledSubsystem;
  int m_r, m_g, m_b;
  LEDPattern m_pattern;
  units::millisecond_t m_period;
  units::millisecond_t m_timeToDisplay;
  units::millisecond_t m_startTime;
};
