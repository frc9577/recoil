// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc/TimedRobot.h>

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;

 private:
  static constexpr int kLength = 300;
  units::time::second_t kPeriod = 1.0_s; 

  // PWM port 9
  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{9};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  // Reuse the buffer

  // Our LED strip has a density of 120 LEDs per meter
  units::meter_t kLedSpacing{1 / 120.0};

  // Create an LED pattern that will display a rainbow across
  // all hues at maximum saturation and half brightness
  frc::LEDPattern m_pattern = frc::LEDPattern::Gradient(frc::LEDPattern::kContinuous, {frc::Color::kGreen, frc::Color::kWhite}).AtBrightness(0.25);
//   frc::LEDPattern m_pattern = frc::LEDPattern::Rainbow(255, 255).AtBrightness(0.1);
//   frc::LEDPattern m_pattern = frc::LEDPattern::Solid(frc::Color::kGreen).AtBrightness(0.1);
//   frc::LEDPattern m_pattern = frc::LEDPattern::Solid(frc::Color::kWhite).Breathe(kPeriod).AtBrightness(0.1);

  // Create a new pattern that scrolls the rainbow pattern across the LED
  // strip, moving at a speed of 1 meter per second.
  frc::LEDPattern m_scrollingPattern =
      m_pattern.ScrollAtAbsoluteSpeed(1_mps, kLedSpacing);

};
