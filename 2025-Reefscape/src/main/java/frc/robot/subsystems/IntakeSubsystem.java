// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Subsystem controlling the mechanisms on the coral intake ramp - alignment motors
// and, possibly, presence sensor(s). This may also need to include the split/drop
// mechanism if and when this is installed to allow for deep climb.

// TODO: Actually implement this. I'm adding a skeleton only so that RobotContainer
// can reference the subsystem.
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private double m_motorSpeed = 0;

  private final SparkMax m_motor = new SparkMax(
    IntakeConstants.kMotorCANID,
    MotorType.kBrushless
  );

  private final DigitalInput m_Sensor = new DigitalInput(
    IntakeConstants.kSensorChannel
  );

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // We need to know if the motor controller we need is
    // actually present on the CAN bus and, unfortunately, the
    // constructor doesn't seem to throw an exception in this case.
    // Let's query for firmware error status and use this for now.
    if (m_motor.getFaults().firmware) {
      throw new RuntimeException("Intake ramp subsystem motor not present");
    }
  }

  public void setMotorSpeed(double speed) {
    m_motor.set(speed);
    m_motorSpeed = speed;
  }

  // Returns the last COMMANDED speed
  public double getMotorSpeed() {
    return m_motorSpeed;
  }

  public boolean isElevatorDown() {
    boolean sensorRead = m_Sensor.get();
    return IntakeConstants.kSensorFalseIsEmpty ? sensorRead : !sensorRead;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {
      /* one-time action goes here */
    });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
