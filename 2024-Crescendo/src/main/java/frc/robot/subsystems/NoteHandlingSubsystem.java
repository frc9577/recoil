// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteHandlingSubsystem extends SubsystemBase {
  private double m_intakeSpeed = 0;
  private double m_outputSpeed = 0;

  private final CANSparkMax m_intakeMotor = new CANSparkMax(NoteHandlingConstants.kIntakeMotorCANID, 
                                                            MotorType.kBrushless);
  private final CANSparkMax m_outputMotor = new CANSparkMax(NoteHandlingConstants.kOutputMotorCANID,
                                                            MotorType.kBrushless);

  private final DigitalInput m_noteSensor = new DigitalInput(NoteHandlingConstants.kNoteSensorChannel);
  
  /** Creates a new NoteHandlingSubsystem. */  
  public NoteHandlingSubsystem() 
  {
    m_intakeMotor.setSmartCurrentLimit(NoteHandlingConstants.kIntakeCurrentLimit);
    m_outputMotor.setSmartCurrentLimit(NoteHandlingConstants.kOutputCurrentLimit);
  }

  public void setIntakeSpeed(double speed)
  {
    m_intakeMotor.set(speed);
    m_intakeSpeed = speed;
  }
  
  // Returns the last COMMANDED speed
  public double getIntakeSpeed()
  {
    return m_intakeSpeed;
  }

  public void setOutputSpeed(double speed)
  {
    m_outputMotor.set(speed);
    m_outputSpeed = speed;
  }

  // Returns the last COMMANDED speed.
  public double getOutputSpeed()
  {
    return m_outputSpeed;
  }

  public boolean hasNote()
  {
    // DIO's on roborio have pullup resistor meaning 
    // they read as 1 when connected switches are not grounded.
    // If a normally-open switch is used they will read as 1 until pressed.
    // if a normally-closed switch is used they will read as 0 until pressed.

    boolean dioValue = m_noteSensor.get();
    return !(dioValue ^ NoteHandlingConstants.kNoteSensorNormallyOpen);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() 
  {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() 
  {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
