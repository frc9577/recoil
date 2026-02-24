// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.factorys.DriveSubsystemFactory;
import frc.robot.factorys.TalonFXFactory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IndexerBulkSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbL1Subsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.AutoCommands;
import frc.robot.utils.HubUtils;
import frc.robot.utils.PneumaticHubWrapper;
import frc.robot.commands.*;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Optional<DriveSubsystem> m_driveSubsystem;
  private final Optional<IntakeSubsystem> m_intakeSubsystem;
  private final Optional<ClimbL1Subsystem> m_climbL1Subsystem;
  private final Optional<IndexerBulkSubsystem> m_indexerBulkSubsystem;
  private final Optional<LauncherSubsystem> m_launcherSubsystem;
  private final Optional<PneumaticHubWrapper> m_pneumaticHub;
  private final LimelightSubsystem m_limelightSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

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
    new Pose2d(0.0, 0.0, new Rotation2d()),
    m_drivetrainError,
    m_limelightError
  );

  PathConstraints m_constraints = new PathConstraints(
          1.0, 
          1.0, 
            (1/2) * Math.PI,
            1 * Math.PI
  ); // The constraints for this path.

  // Smartdashboard Objects
  private SendableChooser<Command> m_autoChooser;
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

    // Init the subsystems that don't require pneumatics.
    m_limelightSubsystem = new LimelightSubsystem(m_PoseEstimator, m_gyro);
    m_launcherSubsystem = getSubsystem(LauncherSubsystem.class);
    m_indexerBulkSubsystem = getSubsystem(IndexerBulkSubsystem.class);

    // Init pneumatics system and subsystems that rely upon it.
    m_pneumaticHub = getSubsystem(PneumaticHubWrapper.class);
    if (m_pneumaticHub.isPresent())
    {
      PneumaticHub hub = m_pneumaticHub.get();
      hub.enableCompressorAnalog(PneumaticsConstants.kMinPneumaticsPressure,
                                 PneumaticsConstants.kMaxPneumaticsPressure);

      m_climbL1Subsystem = getSubsystem(ClimbL1Subsystem.class);
      m_intakeSubsystem = getSubsystem(IntakeSubsystem.class);
    }
    else
    {
      // If there are no pneumatics, there's no climb or intake.
      m_climbL1Subsystem = Optional.empty();
      m_intakeSubsystem  = Optional.empty();
    }
    // Init Auto
    configureAutos();

    // Configure the default commands
    configureDefaultCommands();

    // Configure the trigger bindings
    configureBindings();
  }

  // Tom wrote this cool template to make the optional subsystem creation code in
  // the constructor above a lot clearer. This is what clever coding looks like.
  private static <SSC> Optional<SSC> getSubsystem(Class<SSC> subsystemClass) {
    Optional<SSC> iss;
    try {
      iss = Optional.ofNullable(subsystemClass.getDeclaredConstructor().newInstance());
    } catch (Exception e) {
      iss = Optional.empty();
    }
    return iss;
  }

  // Init Autos (/home/lvuser/deploy/pathplanner/autos)
  // TODO: Check if the new PathPlannerAuto is mirroring the cords when read by the pathfinder.
  private void configureAutos() {
    if (m_driveSubsystem.isPresent()) {
      // Init Needed values
      DriveSubsystem driveSubsystem = m_driveSubsystem.get();
      m_autoChooser = new SendableChooser<Command>();

      // Named Commands
      NamedCommands.registerCommand("RotateTo-180", new RotateToRotation2D(
          driveSubsystem, 
          m_PoseEstimator, 
          Rotation2d.fromDegrees(180), 
          1
        )
      );

      NamedCommands.registerCommand("AimToHub", new AimAtHub(
          driveSubsystem, 
          m_PoseEstimator, 
          2.0, 
          isRed
        )
      );

      // Custom Autos
      m_autoChooser.addOption(
        "Deadreckon 3m at 3.0 mps", 
        new DeadreckonForward(driveSubsystem, 3, 3.0)
      );

      m_autoChooser.addOption(
        "Deadreckon -3m at -1.0", 
        new DeadreckonForward(driveSubsystem, -3, -1.0)
      );

      m_autoChooser.addOption(
        "On-Fly Forward 2m", 
        new DriveForwardFromPos(m_PoseEstimator, 2)
      );

      m_autoChooser.addOption(
        "Rotate to 0", 
        new RotateToRotation2D(
          driveSubsystem, 
          m_PoseEstimator, 
          new Rotation2d(0.0), 
          1
        )
      );

      m_autoChooser.addOption(
        "On-The-Fly to (2, 4.837) -90", 
        new POTFtoPoint(
          m_PoseEstimator, 
          new Pose2d(2, 4.837, Rotation2d.fromDegrees(-90))
        )
      );

      // Path Planner Auto's
      ArrayList<String> autoNames = AutoCommands.getAutoNames();
      for (String autoName : autoNames) {
        PathPlannerAuto plannedAuto = new PathPlannerAuto(autoName);
        Pose2d startingPose = plannedAuto.getStartingPose();

        System.out.println("Normal \"" + autoName + "\" " + startingPose.toString());

        if (isRed.getAsBoolean() == true) {
          startingPose = FlippingUtil.flipFieldPose(startingPose);
          System.out.println("Transposed \"" + autoName + "\" " + startingPose.toString());
        }

        Command rotateToStartRot = new RotateToRotation2D(
          driveSubsystem, 
          m_PoseEstimator,
          startingPose.getRotation(), 
          2.0
        );

        Command pathfindToStartPose = AutoBuilder.pathfindToPose(startingPose, m_constraints);
        Command pathfindThenAuto = Commands.sequence(pathfindToStartPose, rotateToStartRot, plannedAuto);

        m_autoChooser.addOption("[PF] "+autoName, pathfindThenAuto);
      }

      // Add to dashboard
      SmartDashboard.putData("Auto Chooser", m_autoChooser);

      // Field wigit update
      PathPlannerLogging.setLogActivePathCallback((poses) -> {
        m_field.getObject("path").setPoses(poses);
      });
    } else {
      DriverStation.reportWarning("Drive Subsystem is not present! No Auto's configured.", null);
    }
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
    // Set dashboard indicators showing which subsystems are actually present.
    SmartDashboard.putBoolean("Drive Subsystem", m_driveSubsystem.isPresent());
    SmartDashboard.putBoolean("Intake Subsystem", m_intakeSubsystem.isPresent());
    SmartDashboard.putBoolean("ClimbL1 Subsystem", m_climbL1Subsystem.isPresent());
    SmartDashboard.putBoolean("Launcher Subsystem", m_launcherSubsystem.isPresent());
    SmartDashboard.putBoolean("IndexerBulk Subsystem", m_indexerBulkSubsystem.isPresent());

    // Note that some of our command bindings require that multiple subsystems
    // are present. We only enable a binding if all its requirements are met!
    // The command bindings here are implemented based on the 2026 Robot User Manual
    // found at https://docs.google.com/document/d/1VfiFjz9N2ol3pl7xUKzU2Jam5AbL1xXlS_cHFYYanFM

    if (m_driveSubsystem.isPresent()) {
      DriveSubsystem driveSubsystem = m_driveSubsystem.get();

      // Cancel All Drive Commands
      m_driverController.back().onTrue(new CancelDriveCommand(driveSubsystem));

      // Turn left to produce stall error.
      m_driverController.b().onTrue(new TurnLeftTest(driveSubsystem));

      // Aim to Hub
      m_driverController.x().onTrue(
        new AimAtHub(
          driveSubsystem, 
          m_PoseEstimator, 
          2.0, 
          isRed
        )
      );
    }

    //
    // Operator Controls
    //
    if (m_intakeSubsystem.isPresent())
    {
      m_operatorController.rightBumper().onTrue(new StartIntakeCommand(m_intakeSubsystem.get()));
      m_operatorController.leftBumper().onTrue(new StopIntakeCommand(m_intakeSubsystem.get()));
      m_operatorController.povUp().onTrue(new ExtendIntakeCommand(m_intakeSubsystem.get()));
      m_operatorController.povDown().onTrue(new RetractIntakeCommand(m_intakeSubsystem.get()));
    }

    if (m_climbL1Subsystem.isPresent())
    {
      // Operator's manual climb overrides.
      m_operatorController.start().onTrue(new RaiseClimbCommand(m_climbL1Subsystem.get()));
      m_operatorController.back().onTrue(new LowerClimbCommand(m_climbL1Subsystem.get()));    
    }

    if (m_launcherSubsystem.isPresent())
    {
      // Operator's manual launcher override.
      m_operatorController.b().onTrue(new StopLauncherCommand(m_launcherSubsystem.get()));
      
      // FOR TEST PURPOSES ONLY!
      m_operatorController.a().onTrue(new StartFlywheelCommand(m_launcherSubsystem.get(),
                                                               LauncherConstants.kFixedTestSpeed,
                                                               LauncherConstants.kFlywheelToleranceRPM));
      
      // TODO: Need composite command for manual shoot (spin up flywheel, wait for target, start lift motor)
    }

    // TODO: Write and bind composite commands for driver controls.
  }

  // Populate the SmartDashboard on robot init.
  public void InitSmartDashboard() {
    // Non-Subsystem Specific Stuff
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Field", m_field);

    SmartDashboard.putNumber("Target Rotation", 0);
    SmartDashboard.putNumber("Target Angle Diff Abs", 0);
    SmartDashboard.putNumber("Rotation Speed", 0);

    SmartDashboard.putNumber("Hub Distance", HubUtils.getHubDistance(m_PoseEstimator, isRed));
    SmartDashboard.putNumber("mt2 Tag Count", 0.0);
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
    
      SmartDashboard.putNumber("Limelight robotYaw", m_limelightSubsystem.getRobotYaw());
      SmartDashboard.putNumber("Hub Distance", HubUtils.getHubDistance(m_PoseEstimator, isRed));
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

    // Pneumatics compressor
    // if(m_pneumaticHub.isPresent() && ((m_iTickCount % PneumaticsConstants.kTicksPerUpdate) == 0))
    // {
    //   PneumaticHub hub = m_pneumaticHub.get();
    //   SmartDashboard.putNumber("Pressure", hub.getPressure(0));
    //   SmartDashboard.putBoolean("Compressor Running", hub.getCompressor());
    // }
    
    m_iTickCount++;
  }

  // Checks if the robot is on the blue or red alliance. If it cannot get the data it defaults to blue.
  public BooleanSupplier isRed = () -> {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    } else {
      System.out.println("Could not fetch alliance! Defaulting to blue!");
      return false;
    }
  };

  public void disabledInit() {}

  // Move to auto init for competition code.
  public void enabledInit() {
    LimelightHelpers.SetIMUMode("limelight", 4);
    m_gyro.enableLogging(true);
  }

  public void teleopInit() {
    configureDefaultCommands();
  }

  public void autoInit() {
    // This causes issues when auto's do not reset odometry!!
    // if (m_driveSubsystem.isPresent()) {
    //   DriveSubsystem driveSubsystem = m_driveSubsystem.get();

    //   m_gyro.reset();
    //   driveSubsystem.resetPose(new Pose2d());
    // }
  }

  // Gets called every disabled tick.
  public void disabledPeriodic() {
    double cameraYaw = m_limelightSubsystem.getRobotYaw();
    
    m_gyro.enableLogging(false);
    m_gyro.reset();
    m_gyro.setAngleAdjustment(-cameraYaw);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
