// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6;

class Robot : public frc::TimedRobot {
private:

//Initilizes the kraken x44 moter's s1 to canbus id 50 and s2 to canbus id 51.
  CANBus m_canBus{"rio"};
  hardware::TalonFX m_motor_squid1{50, m_canBus};
  hardware::TalonFX m_motor_squid2{51, m_canBus};
  /*
   * Create the control requests once and reuse them every loop. Position
   * control uses gain slot 0 and velocity control uses gain slot 1.
   */
  controls::PositionVoltage m_positionRequest =
      controls::PositionVoltage{0_tr}.WithSlot(0);
  controls::VelocityVoltage m_velocityRequest =
      controls::VelocityVoltage{0_tps}.WithSlot(1);
  controls::NeutralOut m_neutralRequest{};

public:
  Robot();
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
};