// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.factorys.DriveSubsystemFactory;
import frc.robot.factorys.TalonFXFactory;
import frc.robot.subsystems.DriveSubsystem;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
  //    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  private final DifferentialDriveKinematics m_DriveKinematics = new DifferentialDriveKinematics(DrivetrainConstants.trackWidthMeters);

  // Pose Estimators
  private DifferentialDrivePoseEstimator m_DrivePoseEstimator = new DifferentialDrivePoseEstimator(
                                                                  m_DriveKinematics, 
                                                                  m_gyro.getRotation2d(), 
                                                                  0, 
                                                                  0, 
                                                                  new Pose2d());

  // Sendable Choosers
  private SendableChooser<Command> autoChooser;

  // Factorys
  private TalonFXFactory m_TalonFXFactory = new TalonFXFactory();
  private DriveSubsystemFactory m_DriveSubsystemFactory = new DriveSubsystemFactory();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Init DriveSubsystem
    Optional<TalonFX> rightLead = m_TalonFXFactory.construct(DrivetrainConstants.kRightMotorCANID);
    Optional<TalonFX> leftLead = m_TalonFXFactory.construct(DrivetrainConstants.kLeftMotorCANID);
    Optional<TalonFX> rightFollower = m_TalonFXFactory.construct(DrivetrainConstants.kOptionalRightMotorCANID);
    Optional<TalonFX> leftFollower = m_TalonFXFactory.construct(DrivetrainConstants.kOptionalLeftMotorCANID);
    m_driveSubsystem = m_DriveSubsystemFactory.construct(m_DrivePoseEstimator, m_DriveKinematics, m_gyro, rightLead, leftLead, rightFollower, leftFollower);

    // Init the subsystems
    //m_limelightSubsystem = getSubsystem(LimelightSubsystem.class, m_limeLightPoseEstimator);
    //m_exampleSubsystem = getSubsystem(ExampleSubsystem.class);

    // Init Auto
    configureAutos();

    // Configure the default commands
    configureDefaultCommands();

    // Configure the trigger bindings
    configureBindings();
  }

  // Tom wrote this cool template to make the optional subsystem creation code in
  // the constructor above a lot clearer. This is what clever coding looks like.
  // Owen: I added the ability to pass object args through this for dependency injection,
  // I hope this does not break anything or commited a coding sin or somthing
  // private static <SSC> Optional<SSC> getSubsystem(Class<SSC> subsystemClass, Object... args) {
  //   Optional<SSC> iss;
  //   try {

  //     iss = Optional.ofNullable(subsystemClass.getDeclaredConstructor().newInstance(args));
  //   } catch (Exception e) {
  //     iss = Optional.empty();
  //     // This is not tested! - Owen
  //     DriverStation.reportWarning(
  //       String.format(
  //       "The %s was not found!", subsystemClass.getName()), 
  //       false
  //     );
  //   }
  //   return iss;
  // }

  private void configureAutos() {
    // Init Autos (/home/lvuser/deploy/pathplanner/autos)
    ArrayList<Command> autoCommands = AutoCommands.getAutoCommands(m_driveSubsystem);

    // Init Chooser
    autoChooser = AutoBuilder.buildAutoChooser(); // Can make a default by giving a string
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDefaultCommands() {
    if (m_driveSubsystem.isPresent())
    {
      //DriveSubsystem driveSubsystem = m_driveSubsystem.get();
      //driveSubsystem.initDefaultCommand(m_driverController);
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
