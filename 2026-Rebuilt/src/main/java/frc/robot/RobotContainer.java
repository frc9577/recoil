// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.factorys.DriveSubsystemFactory;
import frc.robot.factorys.TalonFXFactory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.IndexerBulkSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimbL1Subsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.HubUtils;
import frc.robot.utils.Pigeon;
import frc.robot.utils.PneumaticHubWrapper;
import frc.robot.commands.*;
import frc.robot.commands.autoCommands.DeadreckonDistance;
import frc.robot.commands.util.CancelDriveCommand;
import frc.robot.utils.LauncherUtils;
import frc.robot.Constants.*;
import frc.robot.Constants.RobotConstants.kStartingNames;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Pigeon m_pigeon;

  // The robot's subsystems and commands are defined here...
  private final Optional<DriveSubsystem> m_driveSubsystem;
  private Optional<IntakeSubsystem> m_intakeSubsystem;
  private Optional<ClimbL1Subsystem> m_climbL1Subsystem;
  private final Optional<IndexerBulkSubsystem> m_indexerBulkSubsystem;
  private final Optional<LauncherSubsystem> m_launcherSubsystem;
  private final Optional<LiftSubsystem> m_liftSubsystem;
  private final Optional<PneumaticHubWrapper> m_pneumaticHub;
  private final LimelightSubsystem m_limelightSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final DifferentialDriveKinematics m_DriveKinematics = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidthMeters);

  // A Static Standard Deviation, in the form of [x, y, theta]ᵀ in meters and radians.
  private Vector<N3> m_drivetrainError = VecBuilder.fill(0.2, 0.2, 0);
  private Vector<N3> m_limelightError = VecBuilder.fill(.7,.7,9999999); // This gets updated per report

  private DifferentialDrivePoseEstimator m_PoseEstimator = new DifferentialDrivePoseEstimator(
    m_DriveKinematics, 
    Rotation2d.fromDegrees(0.0), 
    0, 
    0, 
    new Pose2d(0.0, 0.0, new Rotation2d()),
    m_drivetrainError,
    m_limelightError
  );

  // The general constraints for most paths.
  PathConstraints m_constraints = new PathConstraints(
          2.0, 
          1.0, 
            (1/2) * Math.PI,
            (1/4) * Math.PI
  );

  // Smartdashboard Objects
  private SendableChooser<Command> m_autoChooser;
  private SendableChooser<kStartingNames> m_startingChooser;
  private final Field2d m_field = new Field2d();

  // Factorys
  private TalonFXFactory m_TalonFXFactory = new TalonFXFactory();
  private DriveSubsystemFactory m_DriveSubsystemFactory = new DriveSubsystemFactory();

  // Keep track of time for SmartDashboard updates.
  static int m_iTickCount = 0;

  // Checks if the robot is on the blue or red alliance. If it cannot get the data it defaults to blue.
  public BooleanSupplier m_isRed = () -> {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    } else {
      System.out.println("Could not fetch alliance! Defaulting to blue!");
      return false;
    }
  };

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Init Pigeon
    m_pigeon = new Pigeon(RobotConstants.kPigeon2CANID);

    // Init DriveSubsystem
    Optional<TalonFX> rightLead = m_TalonFXFactory.construct(DrivetrainConstants.kRightMotorCANID);
    Optional<TalonFX> leftLead = m_TalonFXFactory.construct(DrivetrainConstants.kLeftMotorCANID);
    Optional<TalonFX> rightFollower = m_TalonFXFactory.construct(DrivetrainConstants.kOptionalRightMotorCANID);
    Optional<TalonFX> leftFollower = m_TalonFXFactory.construct(DrivetrainConstants.kOptionalLeftMotorCANID);
    m_driveSubsystem = m_DriveSubsystemFactory.construct(m_PoseEstimator, m_DriveKinematics, m_pigeon, rightLead, leftLead, rightFollower, leftFollower, m_isRed);

    // Init the subsystems that don't require pneumatics.
    m_limelightSubsystem = new LimelightSubsystem(m_PoseEstimator);
    m_launcherSubsystem = getSubsystem(LauncherSubsystem.class);
    m_indexerBulkSubsystem = getSubsystem(IndexerBulkSubsystem.class);
    m_liftSubsystem = getSubsystem(LiftSubsystem.class);

    // Init pneumatics system and subsystems that rely upon it.
    m_pneumaticHub = getSubsystem(PneumaticHubWrapper.class);
    Boolean bHasPneumatics = m_pneumaticHub.isPresent();

    if (bHasPneumatics)
    {
      PneumaticHub hub = m_pneumaticHub.get();
      hub.enableCompressorAnalog(PneumaticsConstants.kMinPneumaticsPressure,
                                 PneumaticsConstants.kMaxPneumaticsPressure);
    }

    // Instantiate subsystems that rely upon pneumatics. We want to run
    // the constructors even if the pneumatic hub is not present so that
    // they can create SmartDashboard objects we want to use later.
    try {
      ClimbL1Subsystem climb = new ClimbL1Subsystem(bHasPneumatics);
      m_climbL1Subsystem = Optional.ofNullable(climb);
    } catch (Exception e) {
      m_climbL1Subsystem = Optional.empty();
    }
    
    try {
      IntakeSubsystem intake = new IntakeSubsystem(bHasPneumatics);
      m_intakeSubsystem = Optional.ofNullable(intake);
    } catch (Exception e) {
      m_intakeSubsystem = Optional.empty();
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
  private void configureAutos() {
    if (m_driveSubsystem.isPresent() && m_launcherSubsystem.isPresent() && m_liftSubsystem.isPresent() && m_indexerBulkSubsystem.isPresent() && m_intakeSubsystem.isPresent()) {
      // Init Needed values
      DriveSubsystem driveSubsystem = m_driveSubsystem.get();
      LauncherSubsystem launcherSubsystem = m_launcherSubsystem.get();
      LiftSubsystem liftSubsystem = m_liftSubsystem.get();
      IndexerBulkSubsystem indexerBulkSubsystem = m_indexerBulkSubsystem.get();
      IntakeSubsystem intakeSubsystem = m_intakeSubsystem.get();
      m_autoChooser = new SendableChooser<Command>();

      // Init Autos
      m_autoChooser.setDefaultOption("NONE", new CancelDriveCommand(driveSubsystem));
      m_autoChooser.addOption("Basic backup and shoot",
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              new ExtendIntakeCommand(intakeSubsystem),
              new DeadreckonDistance(driveSubsystem, 1.5, -2.0),
              new AimAtHub(driveSubsystem, m_PoseEstimator, RobotConstants.kRotateToHubSpeed, m_isRed),
              new WaitForFlywheelAtTarget(launcherSubsystem, LauncherConstants.kFlywheelToleranceRPM),
              new StartIntakeCommand(intakeSubsystem),
              new StartShootCommand(liftSubsystem, indexerBulkSubsystem),
              new WaitCommand(3),
              new StopIntakeCommand(intakeSubsystem),
              new WaitCommand(4)
            ),
            new TrackHubFlywheelCommand(launcherSubsystem, m_PoseEstimator, m_isRed)
          ),
          new StopShootCommand(liftSubsystem, indexerBulkSubsystem),
          new StopLauncherCommand(launcherSubsystem)
        )
      );

      // Warm up Pathfinder
      CommandScheduler.getInstance().schedule(
        PathfindingCommand.warmupCommand()
        .andThen(new InstantCommand(
          () -> m_field.getObject("target").setPose(new Pose2d())  
        ))
      );

      // Add to dashboard
      SmartDashboard.putData("Auto Chooser", m_autoChooser);

      // Field wigit update
      PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        m_field.getObject("target").setPose(pose);
      });

      PathPlannerLogging.setLogActivePathCallback((poses) -> {
        m_field.getObject("path").setPoses(poses);
      });
    } else {
      try {
        DriverStation.reportWarning("Drive Subsystem is not present! No Auto's configured.", null);
      } catch (Exception e) {
        // If using the SimGUI with no drivetrain components on the robot, the prior call throws
        // a null pointer exception which is unhelpful. Let's see if this lets us ignore it.
      }

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
    SmartDashboard.putBoolean("Lift Subsystem", m_liftSubsystem.isPresent());
    SmartDashboard.putBoolean("IndexerBulk Subsystem", m_indexerBulkSubsystem.isPresent());
    SmartDashboard.putBoolean("Pneumatic Subsystem", m_pneumaticHub.isPresent());


    // Note that some of our command bindings require that multiple subsystems
    // are present. We only enable a binding if all its requirements are met!
    // The command bindings here are implemented based on the 2026 Robot User Manual
    // found at https://docs.google.com/document/d/1VfiFjz9N2ol3pl7xUKzU2Jam5AbL1xXlS_cHFYYanFM

    if (m_driveSubsystem.isPresent()) {
      DriveSubsystem driveSubsystem = m_driveSubsystem.get();

      // Aim to Hub (TESTING?)
      m_driverController.rightBumper().onTrue(
        new AimAtHub(driveSubsystem, m_PoseEstimator, RobotConstants.kRotateToHubSpeed, m_isRed)
      );

      // Range launcher, turn to face hub and shoot all fuel
      if (m_launcherSubsystem.isPresent()) {
        LauncherSubsystem launcher = m_launcherSubsystem.get();

        // Change speed with operator left y axis when y is being pressed
        // else auto track hub distance
        m_operatorController.y().onTrue(new ManualFlywheelCommand(
          launcher, m_operatorController, m_PoseEstimator, m_isRed));

        if (m_liftSubsystem.isPresent()) {
          LiftSubsystem lift = m_liftSubsystem.get();
          
          if (m_indexerBulkSubsystem.isPresent()) {
            IndexerBulkSubsystem indexer = m_indexerBulkSubsystem.get();

            m_operatorController.x().onTrue(new StartShootCommand(lift, indexer));
            m_operatorController.b().onTrue(new StopShootCommand(lift, indexer));

            m_operatorController.a().onTrue(
              new StopShootCommand(lift, indexer)
              .andThen(new StopLauncherCommand(launcher))
            );

            m_operatorController.rightStick().onFalse(new StopShootCommand(lift, indexer));

            // This is intended to keep shooting until the button is released.
            m_driverController.y().whileTrue(
              new ParallelCommandGroup(
                new TrackHubFlywheelCommand(launcher, m_PoseEstimator, m_isRed),
                new SequentialCommandGroup(
                  new AimAtHub(driveSubsystem, m_PoseEstimator, RobotConstants.kRotateToHubSpeed, m_isRed),
                  new WaitForFlywheelAtTarget(launcher, LauncherConstants.kFlywheelToleranceRPM),
                  new StartShootCommand(lift, indexer)
                )
              )
            );

            m_driverController.y().onFalse(
              new StopShootCommand(lift, indexer)
              .andThen(new StopLauncherCommand(launcher))
            );

            if (m_intakeSubsystem.isPresent()) {
              IntakeSubsystem intake = m_intakeSubsystem.get();
              m_operatorController.rightStick().whileTrue(new ReverseIndexBulkIntake(indexer, intake));
            } else {
              m_operatorController.rightStick().whileTrue(new ReverseIndexBulk(indexer));
            }
          } else {
            m_operatorController.x().onTrue(new StartLiftCommand(lift));
            m_operatorController.b().onTrue(new StopLiftCommand(lift));
          }
        } else {
          m_operatorController.a().onTrue(new StopLauncherCommand(launcher));
        }
      }

      // Travel to corner and shoot
      // m_driverController.x().onTrue(
      //   new TravelToCornerAndShoot(driveSubsystem, m_PoseEstimator, m_isRed, m_constraints)
      // );

      // TEST COMMAND FOR TESTING ACCURACY AFTER BUMP!
      m_driverController.leftBumper().onTrue(
        new RotateToRotation2D(driveSubsystem, m_PoseEstimator, Rotation2d.kZero, 2.0)
      );

      // cancel the current command running on drive subsystem when
      // left y or right x is > half and the default command is not running.
      // new Trigger(() -> (Math.abs(m_driverController.getLeftY()) > 0.5) && (driveSubsystem.getCurrentCommand().getClass() != driveSubsystem.getDefaultCommand().getClass()))
      //   .onTrue(new CancelDriveCommand(driveSubsystem));

      // new Trigger(() -> (Math.abs(m_driverController.getRightX()) > 0.5) && (driveSubsystem.getCurrentCommand().getClass() != driveSubsystem.getDefaultCommand().getClass()))
      //   .onTrue(new CancelDriveCommand(driveSubsystem));

      // false == default, true == not default
      BooleanSupplier isNotDefault = () -> {
        try {
          Command currentCommand = driveSubsystem.getCurrentCommand();
          Command defaultCommand = driveSubsystem.getDefaultCommand();
          if (currentCommand != null && defaultCommand != null) {
            return driveSubsystem.getCurrentCommand().getClass() != driveSubsystem.getDefaultCommand().getClass();
          } else {
            return false;
          }
        } catch (Error e) {
          System.out.println("Error in isNotDefault! Returning false " + e.toString());
          return false;
        }
      };

      new Trigger(() -> (Math.abs(m_driverController.getLeftY()) > 0.5) && (isNotDefault.getAsBoolean()))
        .onTrue(new CancelDriveCommand(driveSubsystem));

      new Trigger(() -> (Math.abs(m_driverController.getRightX()) > 0.5) && (isNotDefault.getAsBoolean()))
        .onTrue(new CancelDriveCommand(driveSubsystem));

       // Also do it on back press
      m_driverController.back().onTrue(new CancelDriveCommand(driveSubsystem));
    }

    m_driverController.start().onTrue(new InstantCommand(() -> {
      Double robotYaw = m_limelightSubsystem.getRobotYaw();
      if (robotYaw != null) {
        Rotation2d currentRotation = m_pigeon.getRotation2d();
        if (Math.abs(robotYaw - currentRotation.getDegrees()) > 1) {
          m_pigeon.reset();
          m_pigeon.setYawOffset(robotYaw);
        }
      }
    }));

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
  }

  // Populate the SmartDashboard on robot init.
  public void InitSmartDashboard() {
    // Non-Subsystem Specific Stuff
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Field", m_field);

    SmartDashboard.putNumber("Target Rotation", 0);
    SmartDashboard.putNumber("Target Angle Diff Abs", 0);
    SmartDashboard.putNumber("Rotation Speed", 0);

    SmartDashboard.putNumber("Hub Distance", HubUtils.getHubDistance(m_PoseEstimator, m_isRed));
    SmartDashboard.putNumber("mt2 Tag Count", 0.0);

    SmartDashboard.putBoolean("Launcher SafeToShoot", false);

    SmartDashboard.putNumber("Yaw", 0.0);
    SmartDashboard.putNumber("Pitch", 0.0);
    SmartDashboard.putNumber("Roll", 0.0);

    SmartDashboard.putNumber("Yaw Rate", 0.0);
    SmartDashboard.putNumber("Pitch Rate", 0.0);
    SmartDashboard.putNumber("Roll Rate", 0.0);

    SmartDashboard.putNumber("Auto Wait Time", 0.0);

    SmartDashboard.putBoolean("Enabled", DriverStation.isEnabled());
    // SmartDashboard.putBoolean("Pidgeon Accurate", m_pigeon.isAccurate());

    m_startingChooser = new SendableChooser<kStartingNames>(); 
    m_startingChooser.setDefaultOption("NONE", null);
    for (kStartingNames startingName : kStartingNames.values()) {
      m_startingChooser.addOption(startingName.name(), startingName);
    }
    SmartDashboard.putData("Starting Position", m_startingChooser);
  }

  // This function is called every 20mS.
  public void periodic() {
    UpdateSmartDashboard();
  }

  //private boolean oldPigeonValue = false;
  private void UpdateSmartDashboard() {
    // Non-subsystem specific stuff
    if ((m_iTickCount % DrivetrainConstants.kTicksPerUpdate) == 0) {
      Pose2d estimatedPos = m_PoseEstimator.getEstimatedPosition();
      m_field.setRobotPose(estimatedPos);

      SmartDashboard.putNumber("Pose X (Meter)", estimatedPos.getX());
      SmartDashboard.putNumber("Pose Y (Meter)", estimatedPos.getY());
      SmartDashboard.putNumber("Pose Theta (Degrees)", estimatedPos.getRotation().getDegrees());
    
      SmartDashboard.putNumber("Yaw", m_pigeon.getYaw());
      SmartDashboard.putNumber("Pitch", m_pigeon.getPitch());
      SmartDashboard.putNumber("Roll", m_pigeon.getRoll());

      SmartDashboard.putNumber("Yaw Rate", m_pigeon.getYawRate());
      SmartDashboard.putNumber("Pitch Rate", 0.0);
      SmartDashboard.putNumber("Roll Rate", 0.0);

      //SmartDashboard.putNumber("Limelight robotYaw", m_limelightSubsystem.getRobotYaw());
      SmartDashboard.putNumber("Hub Distance", HubUtils.getHubDistance(m_PoseEstimator, m_isRed));

      SmartDashboard.putBoolean("Enabled", DriverStation.isEnabled());

      // boolean pigeonAccuracy = m_pigeon.isAccurate();
      // boolean dashboardAccuracy = SmartDashboard.getBoolean("Pidgeon Accurate", pigeonAccuracy);

      // if (pigeonAccuracy != dashboardAccuracy) {
      //   if (pigeonAccuracy == oldPigeonValue) {
      //     m_pigeon.setAccuracy(dashboardAccuracy);
      //     pigeonAccuracy = dashboardAccuracy;
      //   } else {
      //     SmartDashboard.putBoolean("Pidgeon Accurate", pigeonAccuracy);
      //   }
      // }
      // oldPigeonValue = pigeonAccuracy;
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

      SmartDashboard.putNumber("Gyro Degrees", m_pigeon.getYaw());
    }

    // Pneumatics compressor
    if(m_pneumaticHub.isPresent() && ((m_iTickCount % PneumaticsConstants.kTicksPerUpdate) == 0))
    {
      PneumaticHub hub = m_pneumaticHub.get();
      SmartDashboard.putNumber("Pressure", hub.getPressure(0));
      SmartDashboard.putBoolean("Compressor Running", hub.getCompressor());
    }
    
    // Safe-to-shoot indicator.
    if(m_launcherSubsystem.isPresent() && ((m_iTickCount % LauncherConstants.kTicksPerDistanceUpdate) == 0))
    {
      Boolean bCanShoot = LauncherUtils.canScoreFromHere(m_PoseEstimator, m_isRed);
      SmartDashboard.putBoolean("Launcher SafeToShoot", bCanShoot);
    }

    m_iTickCount++;
  }

  // Move to auto init for competition code.
  public void enabledInit() {
    LimelightHelpers.SetIMUMode("limelight", 3);
    m_limelightSubsystem.setAllowJumps(false);
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

  //private int disabledTick = 0;
  public void disabledInit() {
    //disabledTick = 0;
    LimelightHelpers.SetIMUMode("limelight", 3);
    m_limelightSubsystem.setAllowJumps(true);

    // Disable all motors on disabled init
    if (m_driveSubsystem.isPresent()) {
      DriveSubsystem drive = m_driveSubsystem.get();
      drive.setDifferentialSpeedNoPid(0,0);
    }

    if (m_intakeSubsystem.isPresent()) {
      IntakeSubsystem intake = m_intakeSubsystem.get();
      intake.stop();
    }

    if (m_indexerBulkSubsystem.isPresent()) {
      IndexerBulkSubsystem indexBulk = m_indexerBulkSubsystem.get();
      indexBulk.stopBulkTransfer();
      indexBulk.stopIndexer();
    }

    if (m_liftSubsystem.isPresent()) {
      LiftSubsystem lift = m_liftSubsystem.get();
      lift.stopLift();
    }

    if(m_launcherSubsystem.isPresent())
    {
      LauncherSubsystem launcher = m_launcherSubsystem.get();
      launcher.setTargetSpeedrpm(0.0);
    }
  }

  // Gets called every disabled tick.
  private boolean oldIsRed;
  private kStartingNames oldStartEnum;
  public void disabledPeriodic() {
    // Reset Pidgeon
    // if ((disabledTick % 20) == 0) {
    //   Double robotYaw = m_limelightSubsystem.getRobotYaw();
    //   if (robotYaw != null) {
    //     Rotation2d currentRotation = m_pigeon.getRotation2d();
    //     if (Math.abs(robotYaw - currentRotation.getDegrees()) > 1) {
    //       m_pigeon.reset();
    //       m_pigeon.setYawOffset(robotYaw);
    //     }
    //   }
    // }

    // Check the side
    boolean isRed = m_isRed.getAsBoolean();
    kStartingNames startEnum = m_startingChooser.getSelected();
    if (startEnum != null) {
      if ((isRed != oldIsRed) || (startEnum != oldStartEnum)) {
        oldIsRed = isRed;
        oldStartEnum = startEnum;

        Pose2d startingPose = RobotConstants.kStartingPositions.get(startEnum).get(m_isRed.getAsBoolean());

        m_pigeon.reset();
        m_pigeon.setYawOffset(-startingPose.getRotation().getDegrees());

        m_limelightSubsystem.resetYawQue();

        if (m_driveSubsystem.isPresent()) {
          DriveSubsystem driveSubsystem = m_driveSubsystem.get();
          driveSubsystem.resetPose(startingPose);
        } else {
          m_PoseEstimator.resetPosition(
            m_pigeon.getRotation2d(), 
            0, 0, 
            startingPose
          );
        }

        System.out.println("Reset starting pose!");
      }
    } else if(startEnum != oldStartEnum) {
      oldStartEnum = null;
    }

    //disabledTick += 1;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void testInit() {

  }

  public void testPeriodic() {
    if (m_indexerBulkSubsystem.isPresent())
    {
      m_indexerBulkSubsystem.get().testPeriodic();
    }
    if (m_climbL1Subsystem.isPresent())
    {
      m_climbL1Subsystem.get().testPeriodic();
    }
    if (m_intakeSubsystem.isPresent())
    {
      m_intakeSubsystem.get().testPeriodic();
    }
    if (m_launcherSubsystem.isPresent())
    {
      m_launcherSubsystem.get().testPeriodic();
    }
    if (m_liftSubsystem.isPresent())
    {
      m_liftSubsystem.get().testPeriodic();
    }
  }
}
