// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.factorys.DriveSubsystemFactory;
import frc.robot.factorys.TalonFXFactory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.AutoCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Optional<DriveSubsystem> m_driveSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  private final DifferentialDriveKinematics m_DriveKinematics = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidthMeters);

  // A Static Standard Deviation, in the form of [x, y, theta]ᵀ in meters and radians.
  private Vector<N3> m_drivetrainError = VecBuilder.fill(0.2, 0.2, 0);
  private Vector<N3> m_limelightError = VecBuilder.fill(.7,.7,9999999);

  private DifferentialDrivePoseEstimator m_PoseEstimator = new DifferentialDrivePoseEstimator(
    m_DriveKinematics, 
    Rotation2d.fromDegrees(0.0), 
    0, 
    0, 
    new Pose2d(0.0, 8.0, new Rotation2d()),
    m_drivetrainError,
    m_limelightError
  );

  // Smartdashboard Objects
  private SendableChooser<Command> autoChooser;
  private final Field2d m_field = new Field2d();

  // Factorys
  private TalonFXFactory m_TalonFXFactory = new TalonFXFactory();
  private DriveSubsystemFactory m_DriveSubsystemFactory = new DriveSubsystemFactory();

  // Keep track of time for SmartDashboard updates.
  static int m_iTickCount = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Init DriveSubsystem
    Optional<TalonFX> rightLead = m_TalonFXFactory.construct(DrivetrainConstants.kRightMotorCANID);
    Optional<TalonFX> leftLead = m_TalonFXFactory.construct(DrivetrainConstants.kLeftMotorCANID);
    Optional<TalonFX> rightFollower = m_TalonFXFactory.construct(DrivetrainConstants.kOptionalRightMotorCANID);
    Optional<TalonFX> leftFollower = m_TalonFXFactory.construct(DrivetrainConstants.kOptionalLeftMotorCANID);
    m_driveSubsystem = m_DriveSubsystemFactory.construct(m_PoseEstimator, m_DriveKinematics, m_gyro, rightLead, leftLead, rightFollower, leftFollower);

    // Init the subsystems
    m_limelightSubsystem = new LimelightSubsystem(m_PoseEstimator, m_gyro);
    //m_exampleSubsystem = getSubsystem(ExampleSubsystem.class);

    // Init Auto
    configureAutos();

    // Configure the default commands
    configureDefaultCommands();

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureAutos() {
    // Init Autos (/home/lvuser/deploy/pathplanner/autos)
    AutoCommands.getAutoCommands(m_driveSubsystem);

    // Init Chooser
    autoChooser = AutoBuilder.buildAutoChooser(); // Can make a default by giving a string
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDefaultCommands() {
    if (m_driveSubsystem.isPresent())
    {
      DriveSubsystem driveSubsystem = m_driveSubsystem.get();
      driveSubsystem.initDefaultCommand(m_driverController);
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // SmartDashboard.putBoolean("Example Subsystem", m_exampleSubsystem.isPresent());

    // if (m_exampleSubsystem.isPresent())
    // {
    //   ExampleSubsystem exampleSubsystem = m_exampleSubsystem.get();

    //   // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //   new Trigger(exampleSubsystem::exampleCondition)
    //       .onTrue(new ExampleCommand(exampleSubsystem));

    //   // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    //   // cancelling on release.
    //   m_driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
    // }
  }

  // Populate the SmartDashboard on robot init.
  public void InitSmartDashboard() {
    // Non-Subsystem Specific Stuff
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Field", m_field);
  }

  // This function is called every 20mS.  
  public void UpdateSmartDashboard() {
    // Non-subsystem specific stuff
    if ((m_iTickCount % DrivetrainConstants.kTicksPerUpdate) == 0) {
      Pose2d estimatedPos = m_PoseEstimator.getEstimatedPosition();
      m_field.setRobotPose(estimatedPos);

      SmartDashboard.putNumber("Pose X (Meter)", estimatedPos.getX());
      SmartDashboard.putNumber("Pose Y (Meter)", estimatedPos.getY());
      SmartDashboard.putNumber("Pose Theta (Degrees)", estimatedPos.getRotation().getDegrees());
    }

    // Drive subsystem
    if(m_driveSubsystem.isPresent() && (m_iTickCount % DrivetrainConstants.kTicksPerUpdate) == 0)
    {
      DriveSubsystem driveSubsystem = m_driveSubsystem.get();

      SmartDashboard.putNumber("Left Speed (MPS)", driveSubsystem.getMotorSpeedMPS(true));
      SmartDashboard.putNumber("Right Speed (MPS)", driveSubsystem.getMotorSpeedMPS(false));
      SmartDashboard.putNumber("Left Speed (RPS)", driveSubsystem.getMotorSpeedRPS(true));
      SmartDashboard.putNumber("Right Speed (RPS)", driveSubsystem.getMotorSpeedRPS(false));

      SmartDashboard.putNumber("Left Distance (m)", driveSubsystem.getMotorPositionMeters(true));
      SmartDashboard.putNumber("Right Distance (m)", driveSubsystem.getMotorPositionMeters(false));

      SmartDashboard.putNumber("Gyro Degrees", m_gyro.getRotation2d().getDegrees());
    }

    m_iTickCount++;
  }

  public void teleopInit() {
    configureDefaultCommands();
    LimelightHelpers.SetIMUMode("limelight", 4);
  }

  public void autoInit() {
    if (m_driveSubsystem.isPresent()) {
      DriveSubsystem driveSubsystem = m_driveSubsystem.get();

      m_gyro.reset();
      driveSubsystem.resetPose(new Pose2d());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
