// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
// This is a link to do speed control on the kraken motors. We need something like this.
package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.ArcadeDriveCommandNoPID;
import frc.robot.commands.ArcadeFromDashboard;
import frc.robot.commands.DifferentialDriveCommand;
import frc.robot.commands.NoDriveCommand;

public class DriveSubsystem extends SubsystemBase {
  private TalonFX m_rightMotor;
  private TalonFX m_optionalRightMotor;

  private TalonFX m_leftMotor;
  private TalonFX m_optionalLeftMotor; 

  /* Start at velocity 0, use slot 0 */
  private final MotionMagicVelocityVoltage m_leftVelocityVoltage = new MotionMagicVelocityVoltage(0);//.withSlot(0);
  private final MotionMagicVelocityVoltage m_rightVelocityVoltage = new MotionMagicVelocityVoltage(0);//.withSlot(0);

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final AHRS m_gyro;

  private final SendableChooser<Class<?>> m_driveChooser = new SendableChooser<>();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(DifferentialDrivePoseEstimator poseEstimator, DifferentialDriveKinematics kinematics, AHRS gyro, TalonFX rightMotor, TalonFX leftMotor) { 
    m_poseEstimator = poseEstimator;
    m_kinematics = kinematics;
    m_gyro = gyro;
    m_rightMotor = rightMotor;
    m_leftMotor = leftMotor;

    // Right Motor
    setConfig(m_rightMotor, InvertedValue.CounterClockwise_Positive); 
    SendableRegistry.setName(m_rightMotor, "DriveSubsystem", "rightMotor");

    // Left Motor
    setConfig(m_leftMotor, InvertedValue.Clockwise_Positive); 
    SendableRegistry.setName(m_leftMotor, "DriveSubsystem", "leftMotor");

    // Zeroing the encoders
    m_leftMotor.setPosition(0);
    m_rightMotor.setPosition(0); 

    // Init Autobuilder
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> driveRobotRelative(speeds), 
      new PPLTVController(0.02), 
      AutoConstants.kRobotConfig, 
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          DriverStation.Alliance currentAlliance = alliance.get();
          return currentAlliance == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this
    );
    
    // Gyro setup
    m_gyro.reset();

    // Drive Chooser Init
    m_driveChooser.setDefaultOption("PID Arcade", ArcadeDriveCommand.class);
    m_driveChooser.addOption("PID Differential", DifferentialDriveCommand.class);
    m_driveChooser.addOption("NO-PID Arcade", ArcadeDriveCommandNoPID.class);
    m_driveChooser.addOption("Dashboard PID Arcade", ArcadeFromDashboard.class);
    m_driveChooser.addOption("None", NoDriveCommand.class);
    SmartDashboard.putData("Drive Commands", m_driveChooser);

    // Init Smartdashboard
    SmartDashboard.putNumber("DB Left Set (MPS)", 0);
    SmartDashboard.putNumber("DB Right Set (MPS)", 0);
    SmartDashboard.putNumber("DB Arcade Set (MPS)", 0);

    SmartDashboard.putNumber("Left Target (MPS)", 0);
    SmartDashboard.putNumber("Right Target (MPS)", 0);    
    SmartDashboard.putNumber("Left Speed (MPS)", getMotorSpeedMPS(true));
    SmartDashboard.putNumber("Right Speed (MPS)", getMotorSpeedMPS(false));
    SmartDashboard.putNumber("Left Speed (RPS)", getMotorSpeedRPS(true));
    SmartDashboard.putNumber("Right Speed (RPS)", getMotorSpeedRPS(false));

    SmartDashboard.putNumber("Left Set (-1-->1)", 0);
    SmartDashboard.putNumber("Right Set (-1-->1)", 0);
    SmartDashboard.putNumber("Left Get (-1-->1)", 0);
    SmartDashboard.putNumber("Right Get (-1-->1)", 0);  

    SmartDashboard.putNumber("Gyro Degrees", m_gyro.getRotation2d().getDegrees());
  }

  private void setConfig(TalonFX motor, InvertedValue Inverted) {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0.kS = DrivetrainConstants.kS;
    motorConfig.Slot0.kV = DrivetrainConstants.kV;
    motorConfig.Slot0.kA = DrivetrainConstants.kA_linear;
    motorConfig.Slot0.kP = DrivetrainConstants.kP;
    motorConfig.Slot0.kI = DrivetrainConstants.kI; // No output for integrated error
    motorConfig.Slot0.kD = DrivetrainConstants.kD; // A velocity of 1 rps results in 0.1 V output

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = Inverted;

    motorConfig.Voltage.withPeakForwardVoltage(Volts.of(DrivetrainConstants.kPeakVoltage))
    .withPeakReverseVoltage(Volts.of(-DrivetrainConstants.kPeakVoltage));

    var motionMagicConfigs = motorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = DrivetrainConstants.kMotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = DrivetrainConstants.kMotionMagicJerk;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(motorConfig);
      System.out.println("config attempt #" + (i+1) + ": " + status.toString());
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void setFollowers(TalonFX optionalRight, TalonFX optionalLeft) {
    // Right Motor
    m_optionalRightMotor = optionalRight;
  
    setConfig(m_optionalRightMotor, InvertedValue.CounterClockwise_Positive);
    m_optionalRightMotor.setControl(
      new Follower(m_rightMotor.getDeviceID(), MotorAlignmentValue.Aligned)
    );

    // Left Motor
    m_optionalLeftMotor = optionalLeft;

    setConfig(m_optionalLeftMotor, InvertedValue.Clockwise_Positive);
    m_optionalLeftMotor.setControl(
      new Follower(m_leftMotor.getDeviceID(), MotorAlignmentValue.Aligned)
    );
  }

  // This takes the current class selected in the drive chooser and attempts to initilize a command object from it & run it.
  // The commands must have a constructor with a DriveSubsystem and CommandXboxController as args.
  public void initDefaultCommand(CommandXboxController Controller)
  {
    Class<?> driveClass = m_driveChooser.getSelected();

    try {
      Object driveCommand = driveClass.getDeclaredConstructor(DriveSubsystem.class, CommandXboxController.class).newInstance(this, Controller);
      setDefaultCommand((Command) driveCommand);

      System.out.println(String.format("Loaded drive command \"%s\"!", driveClass.getName()));
    } catch (Exception e) {
      DriverStation.reportWarning(
        String.format(
        "Failed to load drive command \"%s\"!", driveClass.getName()), 
        false
      );

      e.printStackTrace();
    }

    // setDefaultCommand(new ArcadeDriveCommand(this, Controller));
  }

  // TODO: Work on a naming scheme for conversion functions

  // This converts rotations of the motor shaft to meters travled.
  public double ConvertRotationsToMeters(double rotations) {
    return rotations * DrivetrainConstants.kDrivetrainGearRatio * DrivetrainConstants.kWheelCircumference;
  }

  // This converts meters travled to rotations of the motor shaft.
  // This is the inverse function of the one above, calculated it myself. - Owen
  public double ConvertMetersToRotations(double meters) {
    return (meters / DrivetrainConstants.kDrivetrainGearRatio) / DrivetrainConstants.kWheelCircumference;
  }

  // This sets the differential speeds based on a desired MPS speed.
  public void setDifferentialSpeeds(double leftSpeedMPS, double rightSpeedMPS)
  {
    //System.out.println("🙀 " + String.valueOf(leftSpeedMPS) + ", " + String.valueOf(rightSpeedMPS));

    SmartDashboard.putNumber("Left Target (MPS)", leftSpeedMPS);
    SmartDashboard.putNumber("Right Target (MPS)", rightSpeedMPS);

    double leftSpeedRPS = ConvertMetersToRotations(leftSpeedMPS);
    double rightSpeedRPS = ConvertMetersToRotations(rightSpeedMPS);

    SmartDashboard.putNumber("Left Target (RPS)", leftSpeedRPS);
    SmartDashboard.putNumber("Right Target (RPS)", rightSpeedRPS);

    m_leftMotor.setControl(m_leftVelocityVoltage.withVelocity(leftSpeedRPS));
    m_rightMotor.setControl(m_rightVelocityVoltage.withVelocity(rightSpeedRPS));
  }

  // pass in -1 to 1 for the motor speed with no closed loop control.
  public void setDifferentialSpeedNoPid(double left, double right) {
    SmartDashboard.putNumber("Left Set (-1-->1)", left);
    SmartDashboard.putNumber("Right Set (-1-->1)", right);  

    m_leftMotor.set(left);
    m_rightMotor.set(right);
  }

  // Wrapping robot position inside of getposition
  public Pose2d getPose(){
    return m_poseEstimator.getEstimatedPosition();
  }

  // Resets the drivetrains pose estimator to zero.
  public void resetPose(Pose2d newPose){
    m_leftMotor.setPosition(0);
    m_rightMotor.setPosition(0);

    m_poseEstimator.resetPosition(
      m_gyro.getRotation2d(), 
      0, 0, 
      newPose
    );
  }

  public double getMotorSpeedRPS(boolean bLeft) 
  {
    double RPS;
    if (bLeft)
    {
      RPS = m_leftMotor.getVelocity().getValueAsDouble();
    }
    else 
    {
      RPS = m_rightMotor.getVelocity().getValueAsDouble();
    }
    return RPS;
  }

  public double getMotorSpeedMPS(boolean bLeft) 
  {
    double RPS = getMotorSpeedRPS(bLeft);
    double MPS = ConvertRotationsToMeters(RPS);
    return MPS;
  }

  // Gets the distance traveled by the motor in meters
  public double getMotorPositionMeters(boolean bLeft) {
    double posRotations;
    if (bLeft == true) {
      posRotations = m_leftMotor.getPosition().getValueAsDouble();
    } else {
      posRotations = m_rightMotor.getPosition().getValueAsDouble();
    }

    double posMeters = ConvertRotationsToMeters(posRotations);
    return posMeters;
  }

  // Returns a robot relative ChassisSpeeds object based on the average linear velocity
  // in meters per second and average angular velocity in radians per second.
  // Currently we are assuming that there is no scale for the motors, we cannot find anywhere to set the scale.
  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    // Linear velocity in meters per second
    double leftMPS = getMotorSpeedMPS(true);
    double rightMPS = getMotorSpeedMPS(false);

    // Make wheelSpeeds object from MPS & converts it to chasis speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftMPS, rightMPS);
    return m_kinematics.toChassisSpeeds(wheelSpeeds);
  }

  // Gives the drivetrain a new drive command based on a robot relative
  // chassis speed object.
  public void driveRobotRelative(ChassisSpeeds relativeChassisSpeed){
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(relativeChassisSpeed);

    setDifferentialSpeeds(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    double lPositionMeters = getMotorPositionMeters(true);
    double rPositionMeters = getMotorPositionMeters(false);
    Rotation2d yaw = m_gyro.getRotation2d();
    //m_gyro.getAngle()

    m_poseEstimator.update(
      yaw, 
      lPositionMeters,
      rPositionMeters
    );

    SmartDashboard.putNumber("Left Get (-1-->1)", m_leftMotor.get());
    SmartDashboard.putNumber("Right Get (-1-->1)", m_rightMotor.get()); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
