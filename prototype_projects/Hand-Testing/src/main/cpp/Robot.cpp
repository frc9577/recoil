// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() {
  /*
   * Create a new TalonFX configuration object. This will store the
   * configuration parameters for the Kraken X44 that we will set below.
   */
  configs::TalonFXConfiguration motorConfig{};

  /*
   * Configure the motor output. Brake mode holds the shaft when the output is
   * zero, and positive output turns the shaft counter-clockwise when looking
   * at the motor face.
   */
  motorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
  motorConfig.MotorOutput.Inverted =
      signals::InvertedValue::CounterClockwise_Positive;

  /*
   * Configure current limits to protect the motor and breaker while testing.
   */
  motorConfig.CurrentLimits.StatorCurrentLimit = 40_A;
  motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  motorConfig.CurrentLimits.SupplyCurrentLimit = 30_A;
  motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  /*
   * Configure the closed loop controller. The Kraken uses its integrated
   * encoder, measured in rotations, so no feedback sensor setup is needed.
   *
   * Set PID values for position control in slot 0.
   * kP is in volts per rotation of error.
   */
  motorConfig.Slot0.kP = 2.4;
  motorConfig.Slot0.kI = 0;
  motorConfig.Slot0.kD = 0.1;

  /*
   * Set PID values for velocity control in slot 1.
   * kV is in volts per rotation per second: roughly 12 V divided by the
   * motor's free speed in rotations per second. Tune on your mechanism.
   */
  motorConfig.Slot1.kS = 0.1;
  motorConfig.Slot1.kV = 0.1;
  motorConfig.Slot1.kP = 0.1;
  motorConfig.Slot1.kI = 0;
  motorConfig.Slot1.kD = 0;

  /*
   * Apply the configuration to the Kraken.
   *
   * Phoenix 6 saves configs on the device automatically, so there is no
   * separate persist step. We retry a few times in case the CAN bus is still
   * coming up when the program starts.
   */
  ctre::phoenix::StatusCode status =
      ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_motor.GetConfigurator().Apply(motorConfig);
    if (status.IsOK()) break;
  }
  frc::SmartDashboard::PutBoolean("Config OK", status.IsOK());

  // Initialize dashboard values
  frc::SmartDashboard::PutBoolean("Motor Enabled", false);
  frc::SmartDashboard::SetDefaultNumber("Target Position", 0);
  frc::SmartDashboard::SetDefaultNumber("Target Velocity", 0);
  frc::SmartDashboard::SetDefaultBoolean("Control Mode", false);
  frc::SmartDashboard::SetDefaultBoolean("Reset Encoder", false);
}

void Robot::RobotPeriodic() {
  // Display encoder position (rotations) and velocity (rotations per second)
  frc::SmartDashboard::PutNumber("Actual Position",
                                 m_motor.GetPosition().GetValue().value());
  frc::SmartDashboard::PutNumber("Actual Velocity",
                                 m_motor.GetVelocity().GetValue().value());

  // Display motor health
  frc::SmartDashboard::PutBoolean("Motor Connected",
                                  m_motor.GetVelocity().GetStatus().IsOK());
  frc::SmartDashboard::PutNumber("Stator Current",
                                 m_motor.GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Temperature",
                                 m_motor.GetDeviceTemp().GetValue().value());

  if (frc::SmartDashboard::GetBoolean("Reset Encoder", false)) {
    frc::SmartDashboard::PutBoolean("Reset Encoder", false);
    // Reset the encoder position to 0
    m_motor.SetPosition(0_tr);
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  /*
   * If the Motor Enabled switch is off, let the motor sit in neutral and
   * skip the closed loop control below.
   */
  if (!frc::SmartDashboard::GetBoolean("Motor Enabled", false)) {
    m_motor.SetControl(m_neutralRequest);
    return;
  }

  if (frc::SmartDashboard::GetBoolean("Control Mode", false)) {
    /*
     * Get the target velocity (rotations per second) from SmartDashboard and
     * set it as the setpoint for the closed loop controller.
     */
    double targetVelocity =
        frc::SmartDashboard::GetNumber("Target Velocity", 0);
    m_motor.SetControl(m_velocityRequest.WithVelocity(
        units::turns_per_second_t{targetVelocity}));
  } else {
    /*
     * Get the target position (number of rotations) from SmartDashboard and set it as
     * the setpoint for the closed loop controller.
     */
    double targetPosition =
        frc::SmartDashboard::GetNumber("Target Position", 0);
    m_motor.SetControl(
        m_positionRequest.WithPosition(units::turn_t{targetPosition}));
  }
}

void Robot::DisabledInit() {
  /*
   * Stop the motor and turn the Motor Enabled switch off whenever the robot
   * is disabled, so it doesn't move the moment the robot is re-enabled.
   */
  m_motor.SetControl(m_neutralRequest);
  frc::SmartDashboard::PutBoolean("Motor Enabled", false);
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif