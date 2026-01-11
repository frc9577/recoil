// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Class header comment. What does this class do?

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorDefaultCommand;

// TODO: This class likely needs to be significantly reworked since we need to be
// able to control the position of the elevator rather than the speed. We'll likely
// use the internal SparkMax position controller to do this so your methods will 
// end up being along the lines of setTargetHeight(), getTargetHeight(), getCurrentHeight(),
// etc.

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(ElevatorConstants.kMotorCANID);

  private final DigitalInput m_LineBreakSensor = new DigitalInput(ElevatorConstants.kSensorChannel);
  private boolean m_sensorBroken = false;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  private double m_TargetHeight = 0.0;

  int levelIndex = 0;
  Double[] levels = {
    ElevatorConstants.kElevatorIntakePosition,
    ElevatorConstants.kElevatorL2Position,
    ElevatorConstants.kElevatorL3Position,
    ElevatorConstants.kElevatorL4Position
  };

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // We need to know if the motor controller we need is
    // actually present on the CAN bus and, unfortunately, the 
    // constructor doesn't seem to throw an exception in this case. 
    // Let's query for firmware error status and use this for now.

    if (m_motor.isConnected() == false)
    {
      throw new RuntimeException("Elevator subsystem motor not present");
    }

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.Slot0.kV = ElevatorConstants.kV;
    configs.Slot0.kS = ElevatorConstants.kS;
    configs.Slot0.kP = ElevatorConstants.kP;
    configs.Slot0.kI = ElevatorConstants.kI; // No output for integrated error
    configs.Slot0.kD = ElevatorConstants.kD; // A velocity of 1 rps results in 0.1 V output
    
    configs.Voltage.withPeakForwardVoltage(Volts.of(ElevatorConstants.PeakVoltage))
      .withPeakReverseVoltage(Volts.of(-ElevatorConstants.PeakVoltage));

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // Make sure we start at the home position.
    m_motor.setPosition(0.0);
  }

  // This is assuming that postive speeds is moving the elevator up
  public void setMotorSpeed(double speed)
  {
    if (ElevatorConstants.kPositiveMovesUp) {
      m_motor.set(speed);
    } else {
      m_motor.set(-speed);
    }
  }
  
  // Returns the current speed with postive speeds indicating upward movment
  public double getMotorSpeed()
  {
    if (ElevatorConstants.kPositiveMovesUp) {
      return m_motor.get();
    } else {
      return -m_motor.get();
    }
  }

  // Get what the controllers level index is set to
  public int getElevatorLevelIndex() {
    return levelIndex;
  }

  // Change what pre-set level the elevator is at.
  public void changeElevatorLevel(int indexChange) {
    int newLevelIndex = levelIndex + indexChange;
    newLevelIndex = Math.min(Math.max(newLevelIndex, 0), (levels.length-1));

    setTargetPosition(levels[newLevelIndex]);
    levelIndex = newLevelIndex;
  }

  // Sets what pre-set level the elevator is at.
  public void setElevatorLevel(int newLevelIndex) {
    newLevelIndex = Math.min(Math.max(newLevelIndex, 0), (levels.length-1));

    setTargetPosition(levels[newLevelIndex]);
    levelIndex = newLevelIndex;
  }

  public boolean isElevatorDown()
  {
    boolean sensorRead = m_LineBreakSensor.get();
    return ElevatorConstants.kSensorFalseIsEmpty ? sensorRead : !sensorRead;
  }

  // Set the target elevator height in metres above the base position.
  public void setTargetPosition(double height)
  {
    m_TargetHeight = height;

    double elevatorPositionRotations = height / ElevatorConstants.kElevatorGearRatio;
    if (!ElevatorConstants.kPositiveMovesUp)
    {
      elevatorPositionRotations = -elevatorPositionRotations;
    }

    m_motor.setControl(m_positionVoltage.withPosition(elevatorPositionRotations));
  }

  // Return the current setpoint of the elevator in metres above the base position.
  public double getTargetPosition()
  {
    return m_TargetHeight;
  }

  // Returns the height of the elevator from the base.
  public double getPosition()
  {
    double elevatorPosition = m_motor.getPosition().getValueAsDouble() * ElevatorConstants.kElevatorGearRatio; 

    if (ElevatorConstants.kPositiveMovesUp) {
      return elevatorPosition;
    } else {
      return -elevatorPosition;
    }
  }
  
  // Wrapper function for all of the zeroing stuff.
  public void zeroPosition() {
    setTargetPosition(0.0);
    m_motor.setPosition(0);
    levelIndex = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean elevatorDown = isElevatorDown();

    if ((elevatorDown == true) && (m_sensorBroken == false)) {
      zeroPosition();
      m_sensorBroken = true;
    }

    if ((elevatorDown == false) && (m_sensorBroken == true)) {
      m_sensorBroken = false;
    }
  }

  public void initDefaultCommand(XboxController operatorController)
  {
    setDefaultCommand(new ElevatorDefaultCommand(this, operatorController));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
