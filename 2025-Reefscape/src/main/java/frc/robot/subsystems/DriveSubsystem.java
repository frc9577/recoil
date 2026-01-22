// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Class header comment. What does this class do and what assumptions
// (if any) is it making?

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.TankDriveCommand;

public class DriveSubsystem extends SubsystemBase {

  private final TalonFX m_rightMotor = new TalonFX(
    DrivetrainConstants.kRightMotorCANID
  );
  private TalonFX m_optionalRightMotor;

  private final TalonFX m_leftMotor = new TalonFX(
    DrivetrainConstants.kLeftMotorCANID
  );
  private TalonFX m_optionalLeftMotor;

  private DifferentialDrive m_Drivetrain;

  private double m_leftSpeed = 0.0;
  private double m_rightSpeed = 0.0;

  private double m_modeMultiplier = 1.0;

  public class pidControl {

    private TalonFX m_motor;
    private boolean m_positiveMovesForward;

    private double m_targetPosition;

    private final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        DrivetrainConstants.maxVelocity,
        DrivetrainConstants.maxAcceleration
      )
    );

    private PositionVoltage m_positionVoltage;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setPoint = new TrapezoidProfile.State();

    public pidControl(TalonFX motor, boolean positiveMovesForward) {
      m_motor = motor;
      m_positiveMovesForward = positiveMovesForward;
    }

    public void setTargetPosition(double position) {
      m_targetPosition = position;

      double positionRotations =
        position * DrivetrainConstants.kDrivetrainGearRatio;
      if (!m_positiveMovesForward) {
        positionRotations = -positionRotations;
      }

      m_goal = new TrapezoidProfile.State(positionRotations, 0);
      m_setPoint = new TrapezoidProfile.State(0, 0);

      m_positionVoltage = new PositionVoltage(0).withSlot(0);

      m_motor.setPosition(0); // zero's motor at the start of the movement
    }

    public void calculatePosition() {
      m_setPoint = m_profile.calculate(0.020, m_setPoint, m_goal);

      m_positionVoltage.Position = m_setPoint.position;
      m_positionVoltage.Velocity = m_setPoint.velocity;
      m_motor.setControl(m_positionVoltage);
    }

    public double getTargetPosition() {
      return m_targetPosition;
    }

    public double getPosition() {
      double motorPosition =
        m_motor.getPosition().getValueAsDouble() /
        DrivetrainConstants.kDrivetrainGearRatio;

      if (m_positiveMovesForward) {
        return motorPosition;
      } else {
        return -motorPosition;
      }
    }
  }

  pidControl m_leftPidControl = new pidControl(
    m_leftMotor,
    DrivetrainConstants.kLeftPositiveMovesForward
  );

  pidControl m_rightPidControl = new pidControl(
    m_rightMotor,
    DrivetrainConstants.kRightPositiveMovesForward
  );

  //The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Right Motor
    TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
    rightMotorConfig.Slot0.kV = DrivetrainConstants.kV;
    rightMotorConfig.Slot0.kS = DrivetrainConstants.kS;
    rightMotorConfig.Slot0.kP = DrivetrainConstants.kP;
    rightMotorConfig.Slot0.kI = DrivetrainConstants.kI; // No output for integrated error
    rightMotorConfig.Slot0.kD = DrivetrainConstants.kD; // A velocity of 1 rps results in 0.1 V output

    rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightMotorConfig.Voltage.withPeakForwardVoltage(
      Volts.of(DrivetrainConstants.PeakVoltage)
    ).withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

    m_rightMotor.getConfigurator().apply(rightMotorConfig);
    SendableRegistry.setName(m_rightMotor, "DriveSubsystem", "rightMotor");

    // Optional Right Motor
    try {
      m_optionalRightMotor = new TalonFX(
        DrivetrainConstants.kOptionalRightMotorCANID
      );

      // Setting up Config
      TalonFXConfiguration optionalRightMotorConfig =
        new TalonFXConfiguration();
      optionalRightMotorConfig.Slot0.kV = DrivetrainConstants.kV;
      optionalRightMotorConfig.Slot0.kS = DrivetrainConstants.kS;
      optionalRightMotorConfig.Slot0.kP = DrivetrainConstants.kP;
      optionalRightMotorConfig.Slot0.kI = DrivetrainConstants.kI; // No output for integrated error
      optionalRightMotorConfig.Slot0.kD = DrivetrainConstants.kD; // A velocity of 1 rps results in 0.1 V output

      optionalRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      optionalRightMotorConfig.MotorOutput.Inverted =
        InvertedValue.Clockwise_Positive;

      optionalRightMotorConfig.Voltage.withPeakForwardVoltage(
        Volts.of(DrivetrainConstants.PeakVoltage)
      ).withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

      // Saving
      m_optionalRightMotor.getConfigurator().apply(optionalRightMotorConfig);

      // Setting as a follwer
      m_optionalRightMotor.setControl(
        new Follower(m_rightMotor.getDeviceID(), false)
      );
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Left Motor
    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    leftMotorConfig.Slot0.kV = DrivetrainConstants.kV;
    leftMotorConfig.Slot0.kS = DrivetrainConstants.kS;
    leftMotorConfig.Slot0.kP = DrivetrainConstants.kP;
    leftMotorConfig.Slot0.kI = DrivetrainConstants.kI; // No output for integrated error
    leftMotorConfig.Slot0.kD = DrivetrainConstants.kD; // A velocity of 1 rps results in 0.1 V output

    leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftMotorConfig.MotorOutput.Inverted =
      InvertedValue.CounterClockwise_Positive;

    leftMotorConfig.Voltage.withPeakForwardVoltage(
      Volts.of(DrivetrainConstants.PeakVoltage)
    ).withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

    m_leftMotor.getConfigurator().apply(leftMotorConfig);
    SendableRegistry.setName(m_leftMotor, "DriveSubsystem", "leftMotor");

    // Optional Left Motor
    try {
      m_optionalLeftMotor = new TalonFX(
        DrivetrainConstants.kOptionalLeftMotorCANID
      );

      // Setting up Config
      TalonFXConfiguration optionalLeftMotorConfig = new TalonFXConfiguration();
      optionalLeftMotorConfig.Slot0.kV = DrivetrainConstants.kV;
      optionalLeftMotorConfig.Slot0.kS = DrivetrainConstants.kS;
      optionalLeftMotorConfig.Slot0.kP = DrivetrainConstants.kP;
      optionalLeftMotorConfig.Slot0.kI = DrivetrainConstants.kI; // No output for integrated error
      optionalLeftMotorConfig.Slot0.kD = DrivetrainConstants.kD; // A velocity of 1 rps results in 0.1 V output

      optionalLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      optionalLeftMotorConfig.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive;

      optionalLeftMotorConfig.Voltage.withPeakForwardVoltage(
        Volts.of(DrivetrainConstants.PeakVoltage)
      ).withPeakReverseVoltage(Volts.of(-DrivetrainConstants.PeakVoltage));

      // Saving
      m_optionalLeftMotor.getConfigurator().apply(optionalLeftMotorConfig);

      // Setting as follower
      m_optionalLeftMotor.setControl(
        new Follower(m_leftMotor.getDeviceID(), false)
      );
    } catch (Exception e) {
      // TODO: There are really two cases you want to catch. The first, when the follower
      // motor controller doesn't exist, isn't an error. The second, where the motor exists
      // but one of the later configuration calls fails, is an error. Generally, you would
      // only dump a stack trace in error cases and you definitely don't want to do this in
      // normal operation. I would suggest splitting this block into two try/except chunks,
      // on that catches the missing controller and just outputs a status message to the log
      // indicating that only one motor is in use, and the other catching the real errors
      // and dumping the stack trace.
      e.printStackTrace();
    }

    // Zeroing the encoders
    m_leftMotor.setPosition(0);
    m_rightMotor.setPosition(0);

    // Setting up the drive train
    m_Drivetrain = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    SendableRegistry.setName(m_Drivetrain, "DriveSubsystem", "Drivetrain");

    // Gyro setup
    m_gyro.zeroYaw();
  }

  public void initDefaultCommand(XboxController Controller, boolean isArcade) {
    if (isArcade == true) {
      setDefaultCommand(new ArcadeDriveCommand(this, Controller));
    } else {
      setDefaultCommand(new TankDriveCommand(this, Controller));
    }
  }

  // Sets left and right motors to set speeds to support tank drive models.
  public void setTankSpeeds(double leftInput, double rightInput) {
    m_leftSpeed =
      (leftInput / DrivetrainConstants.kSpeedDivider) * m_modeMultiplier;
    m_rightSpeed =
      (rightInput / DrivetrainConstants.kSpeedDivider) * m_modeMultiplier;

    // NOTE: We are squaring the input to improve driver response
    m_Drivetrain.tankDrive(m_leftSpeed, m_rightSpeed, true);
  }

  public void setArcadeSpeeds(double joystickInput, double rotationInput) {
    m_leftSpeed =
      (joystickInput / DrivetrainConstants.kSpeedDivider) * m_modeMultiplier;
    m_rightSpeed = (rotationInput / DrivetrainConstants.kTurnDivider);

    // NOTE: We are making our own custom input modifications
    //m_leftSpeed = Math.pow(m_leftSpeed, 3);
    //m_rightSpeed = Math.pow(m_rightSpeed, 3);

    m_Drivetrain.arcadeDrive(m_leftSpeed, m_rightSpeed, true);
  }

  // Changes controls to normal or reverse mode's
  public void setReverseMode(boolean reverseMode) {
    m_modeMultiplier = reverseMode ? -1.0 : 1.0;
    SmartDashboard.putBoolean("Reverse Mode", reverseMode);
  }

  public double getSpeed(boolean bLeft) {
    if (bLeft) {
      return m_leftSpeed;
    } else {
      return m_rightSpeed;
    }
  }

  // For auto, sets both motors to the target position
  public void setTargetPositionStraight(double position) {
    m_leftPidControl.setTargetPosition(position);
    m_rightPidControl.setTargetPosition(position);
  }

  // For Auto, sets the selected motors TARGET position
  public void setTargetPosition(double position, boolean isLeft) {
    if (isLeft) {
      m_leftPidControl.setTargetPosition(position);
    } else {
      m_rightPidControl.setTargetPosition(position);
    }
  }

  // For Auto, returns the selected motors TARGET position
  public double getTargetPosition(boolean isLeft) {
    if (isLeft) {
      return m_leftPidControl.getTargetPosition();
    } else {
      return m_rightPidControl.getTargetPosition();
    }
  }

  public float getGyroYaw() {
    return m_gyro.getYaw();
  }

  // For Auto, returns the selected motors CURRENT position
  public double getPosition(boolean isLeft) {
    if (isLeft) {
      return m_leftPidControl.getPosition();
    } else {
      return m_rightPidControl.getPosition();
    }
  }

  // For Auto, returns if the left and right motor are within tolerance of the PID target
  public boolean isAtTarget(double tolerance) {
    double leftError = Math.abs(
      m_leftPidControl.getTargetPosition() - m_leftPidControl.getPosition()
    );
    double rightError = Math.abs(
      m_rightPidControl.getTargetPosition() - m_rightPidControl.getPosition()
    );

    return (leftError <= tolerance) && (rightError <= tolerance);
  }

  // For Auto, needs to be in an auto command execute, so we can use the PID without
  // differential drive timeing out.
  public void callDrivetrainFeed() {
    m_Drivetrain.feed();
  }

  public void callCalculate() {
    m_leftPidControl.calculatePosition();
    m_rightPidControl.calculatePosition();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
