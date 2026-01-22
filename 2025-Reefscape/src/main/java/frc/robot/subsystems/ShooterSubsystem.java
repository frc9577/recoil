// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private double m_intakeSpeed = 0;
  private double m_outputSpeed = 0;

  private final SparkMax m_intakeMotor = new SparkMax(
    ShooterConstants.kIntakeMotorCANID,
    MotorType.kBrushless
  );
  private final SparkMax m_outputMotor = new SparkMax(
    ShooterConstants.kOutputMotorCANID,
    MotorType.kBrushless
  );

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // We need to know if the motor controllers we need are
    // actually present on the CAN bus and, unfortunately, their
    // constructors don't seem to throw exceptions in this case. Let's
    // query the firmware fault status and use this for now.
    if (
      m_intakeMotor.getFaults().firmware || m_outputMotor.getFaults().firmware
    ) {
      throw new RuntimeException("Shooter subsystem motors not present");
    }
  }

  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(speed);
    m_intakeSpeed = speed;
  }

  // Returns the last COMMANDED speed
  public double getIntakeSpeed() {
    return m_intakeSpeed;
  }

  public void setOutputSpeed(double speed) {
    m_outputMotor.set(speed);
    m_outputSpeed = speed;
  }

  // Returns the last COMMANDED speed.
  public double getOutputSpeed() {
    return m_outputSpeed;
  }
}
