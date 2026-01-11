// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AutonomousDrivePID;
import frc.robot.commands.AutonomousTurnDegrees;
import frc.robot.commands.ChangeElevatorLevel;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOutputCommand;
import frc.robot.commands.CoralSpeedCommand;
import frc.robot.commands.ElevatorManualSpeed;
import frc.robot.commands.EmptyCommand;
import frc.robot.commands.IntakeAlgaeCommand;
import frc.robot.commands.OutputAlgaeCommand;
import frc.robot.commands.SetElevatorLevel;
import frc.robot.commands.SetModeCommand;
import frc.robot.commands.TimedCommand;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.Optional;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> m_isArcadeChooser = new SendableChooser<>();

  private final PneumaticHub   m_pnuematicHub   = new PneumaticHub();

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final Optional<IntakeSubsystem> m_intakeSubsystem;
  private final Optional<CoralSubsystem> m_coralSubsystem;
  private final Optional<ElevatorSubsystem> m_elevatorSubsystem;
  private final Optional<AlgaeSubsystem> m_algaeSubsystem;

  // Joysticks
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);

  private final JoystickButton m_enableReverse =
    new JoystickButton(m_driverController, OperatorConstants.kEnableReverse);
  private final JoystickButton m_disableReverse = 
    new JoystickButton(m_driverController, OperatorConstants.kDisableReverse);

  private final JoystickButton m_zeroButton =
    new JoystickButton(m_operatorController, OperatorConstants.kZeroElevator);
  private final JoystickButton m_increaseElevatorLevelButton =
    new JoystickButton(m_operatorController, OperatorConstants.kIncreaseElevatorLevel);
  private final JoystickButton m_decreeseElevatorlevelButton =
    new JoystickButton(m_operatorController, OperatorConstants.kDecreeseElevatorLevel);
  private final JoystickButton m_IntakeElevatorPositionButton =
    new JoystickButton(m_operatorController, OperatorConstants.kIntakeElevatorPosition);

  private final JoystickButton m_algaeIntakeButton =
    new JoystickButton(m_operatorController, OperatorConstants.kAlageIntake);
  private final JoystickButton m_algaeOutputButton =
    new JoystickButton(m_operatorController, OperatorConstants.kAlageOutput);

  private final JoystickButton m_coralIntakeButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kCoralIntake);
  private final JoystickButton m_coralOutputButton =
    new JoystickButton(m_operatorController, OperatorConstants.kCoralOuttake);
  private final JoystickButton m_coralStopMotorsButton =
    new JoystickButton(m_operatorController, OperatorConstants.kCoralStop);

  private ElevatorManualSpeed m_manualSpeedCommand;

  // Keep track of time for SmartDashboard updates.
  static int m_iTickCount = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_pnuematicHub.enableCompressorAnalog(RobotConstants.minPnuematicsPressure, RobotConstants.maxPnuematicsPressure);

    m_intakeSubsystem = getSubsystem(IntakeSubsystem.class);
    m_coralSubsystem = getSubsystem(CoralSubsystem.class);
    m_elevatorSubsystem = getSubsystem(ElevatorSubsystem.class); 
    m_algaeSubsystem = getSubsystem(AlgaeSubsystem.class); 

    // Set up SmartDashboard/Shuffleboard widgets for driver/operator use.
    configureDriverStationControls();

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
      //TODO Emit a warning to the driver station that the intake subsystem is not present.
    }
    return iss;
  }

  //
  // Configure all options that we want to display on the Shuffleboard dashboard.
  //
  private void configureDriverStationControls(){
    // Drop-down chooser for auto program.
    m_autoChooser.setDefaultOption("(PID) Drive forward 2 meters", 
      new AutonomousDrivePID(m_driveSubsystem, 2.0, 2.0, 0.04)
    );
    m_autoChooser.addOption("(PID) Drive forward 5 meters", 
      new AutonomousDrivePID(m_driveSubsystem, 5.0, 5.0, 0.04)
    );
    m_autoChooser.addOption("(PID) Drive backward 2 meters", 
      new AutonomousDrivePID(m_driveSubsystem, -2.0, -2.0, 0.04)
    );
    m_autoChooser.addOption("(PID) Drive backward 5 meters", 
      new AutonomousDrivePID(m_driveSubsystem, -5.0, -5.0, 0.04)
    );

    m_autoChooser.addOption("(PID) Drive forward 2 meters then backward 2 meters",
      new AutonomousDrivePID(m_driveSubsystem, 2.0, 2.0, 0.04)
      .andThen(new TimedCommand(new EmptyCommand(), 1000))
      .andThen(new AutonomousDrivePID(m_driveSubsystem, -2.0, -2.0, 0.04))
    );

    m_autoChooser.addOption("(Gyro) Turn 90 Degrees clockwise", 
      new AutonomousTurnDegrees(m_driveSubsystem, 90, 0.3)
    );

    m_autoChooser.addOption("(Gyro) Turn 90 Degrees counter-clockwise", 
      new AutonomousTurnDegrees(m_driveSubsystem, -90, 0.3)
    );

    m_autoChooser.addOption("(Gyro) Turn 90 degrees clockwise then 90 degres counter-clockwise", 
      new AutonomousTurnDegrees(m_driveSubsystem, 90, 0.3)
      .andThen(new TimedCommand(new EmptyCommand(), 1000))
      .andThen(new AutonomousTurnDegrees(m_driveSubsystem, -90, 0.3))
    );

    m_autoChooser.addOption("(Combined) Drive from blue most right to reef",
      new AutonomousDrivePID(m_driveSubsystem, 1.517, 1.517, 0.04)
      .andThen(new TimedCommand(new EmptyCommand(), 1000))
      .andThen(new AutonomousTurnDegrees(m_driveSubsystem, 30, 0.3))
      .andThen(new TimedCommand(new EmptyCommand(), 1000))
      .andThen(new AutonomousDrivePID(m_driveSubsystem, 2.369, 2.369, 0.04))
    );

    m_isArcadeChooser.setDefaultOption("Arcade Drive", true);
    m_isArcadeChooser.addOption("Tank Drive", false);

    SmartDashboard.putData(m_autoChooser);
    SmartDashboard.putData(m_isArcadeChooser);
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

    // TODO: Currently, the following assumes that we're running simple commands that
    // don't depend upon more than a single subsystem. It's very likely that we'll have
    // complex commands that operate one subsystem but need to query state from another
    // (for example, maybe the intake command has to wait for the elevator to get to a
    // known position before feeding coral into it). That will require different logic
    // to make sure that both dependent subsystems are valid before the binding is added.

    SmartDashboard.putBoolean("Intake Subsystem", m_intakeSubsystem.isPresent());
    SmartDashboard.putBoolean("Coral Subsystem", m_coralSubsystem.isPresent());
    SmartDashboard.putBoolean("Elevator Subsystem", m_elevatorSubsystem.isPresent());
    SmartDashboard.putBoolean("Algae Subsystem", m_algaeSubsystem.isPresent());

    if (m_intakeSubsystem.isPresent())
    {
      // Bind operator controls related to the coral subsystem only if it is present on
      // the robot.

      // TODO: Bind intake commands and controls.
    }

    if (m_coralSubsystem.isPresent())
    {
      CoralSubsystem coralSubsystem = m_coralSubsystem.get();
      // Bind operator controls related to the coral subsystem only if it is present on
      // the robot.

      m_coralIntakeButton.onTrue(new CoralIntakeCommand(coralSubsystem));
      m_coralOutputButton.onTrue(new CoralOutputCommand(coralSubsystem));

      m_coralStopMotorsButton.onTrue(
        new CoralSpeedCommand(coralSubsystem, 0, 0));
    }

    if (m_elevatorSubsystem.isPresent())
    {
      ElevatorSubsystem elevatorSubsystem = m_elevatorSubsystem.get();
      // Bind operator controls related to the elevator subsystem only if it is present on
      // the robot.

      m_zeroButton.onTrue(new ZeroElevator(elevatorSubsystem));
      m_increaseElevatorLevelButton.onTrue(
        new ChangeElevatorLevel(elevatorSubsystem, 1));
      m_decreeseElevatorlevelButton.onTrue(
        new ChangeElevatorLevel(elevatorSubsystem, -1));
      m_IntakeElevatorPositionButton.onTrue(
        new SetElevatorLevel(elevatorSubsystem, 0));
    }

    if (m_algaeSubsystem.isPresent())
    {
      AlgaeSubsystem algaeSubsystem = m_algaeSubsystem.get();
      // Bind operator controls related to the algae subsystem only if it is present on
      // the robot.

      m_algaeIntakeButton.onTrue(new IntakeAlgaeCommand(algaeSubsystem));
      m_algaeOutputButton.onTrue(new OutputAlgaeCommand(algaeSubsystem));
    }

    // Drive subsystem
    m_enableReverse.onTrue(new SetModeCommand(m_driveSubsystem, true));
    m_disableReverse.onTrue(new SetModeCommand(m_driveSubsystem, false));
  }

  private void configureDefaultCommands() {
    if (m_elevatorSubsystem.isPresent()) {
      ElevatorSubsystem elevatorSubsystem = m_elevatorSubsystem.get();
      
      elevatorSubsystem.initDefaultCommand(m_operatorController);;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void setDriveType() {
    m_driveSubsystem.initDefaultCommand(
      m_driverController, 
      m_isArcadeChooser.getSelected()
    );
  }

  public void UpdateSmartDashboard() {
    // TODO: Use m_iTickCount to determine which status updates to send back to
    // the driver station. This function is called every 20mS.

    // Drive subsystem (always present)
    if((m_iTickCount % Constants.DrivetrainConstants.kTicksPerUpdate) == 0)
    {
      SmartDashboard.putNumber("Left Encoder", m_driveSubsystem.getPosition(true));
      SmartDashboard.putNumber("Left Target", m_driveSubsystem.getTargetPosition(true));
      SmartDashboard.putNumber("Left Speed", m_driveSubsystem.getSpeed(true));

      SmartDashboard.putNumber("Right Encoder", m_driveSubsystem.getPosition(false));
      SmartDashboard.putNumber("Right Target", m_driveSubsystem.getTargetPosition(false));
      SmartDashboard.putNumber("Right Speed", m_driveSubsystem.getSpeed(false));

      SmartDashboard.putNumber("Gyro Yaw", m_driveSubsystem.getGyroYaw());
    }

    // Coral subsystem state update.
    if(m_coralSubsystem.isPresent() && (m_iTickCount % Constants.CoralConstants.kTicksPerUpdate) == 0)
    {
      SmartDashboard.putBoolean("Has Coral", m_coralSubsystem.get().hasCoral());
      SmartDashboard.putNumber("Input Speed", m_coralSubsystem.get().getIntakeSpeed());
      SmartDashboard.putNumber("Output Speed", m_coralSubsystem.get().getOutputSpeed());
    }

    // Elevator subsystem state update.
    if(m_elevatorSubsystem.isPresent() && (m_iTickCount % Constants.ElevatorConstants.kTicksPerUpdate) == 0)
    {
      ElevatorSubsystem elevatorSubsystem = m_elevatorSubsystem.get();

      SmartDashboard.putNumber("Elevator Position", elevatorSubsystem.getPosition());
      SmartDashboard.putNumber("Elevator Set Point", elevatorSubsystem.getTargetPosition());
      SmartDashboard.putNumber("Elevator Speed", elevatorSubsystem.getMotorSpeed());
      SmartDashboard.putNumber("Elevator Level Index", elevatorSubsystem.getElevatorLevelIndex());

      SmartDashboard.putBoolean("Elevator Down", elevatorSubsystem.isElevatorDown());
    }
    
    // Intake subsystem state update.
    if(m_intakeSubsystem.isPresent() && (m_iTickCount % Constants.IntakeConstants.kTicksPerUpdate) == 0)
    {
      // TODO: Do intake stuff here.
    }

    // Compressor
    if ((m_iTickCount % Constants.RobotConstants.kpnuematicsTicksPerUpdate) == 0) {
      SmartDashboard.putNumber("Pressure", m_pnuematicHub.getPressure(0));
      SmartDashboard.putBoolean("Compressor Running", m_pnuematicHub.getCompressor());
    }

    // Algae subsystem state update.
    if(m_algaeSubsystem.isPresent() && (m_iTickCount % Constants.AlgaeConstants.kTicksPerUpdate) == 0)
    {
      SmartDashboard.putBoolean("Has Algae", m_algaeSubsystem.get().hasAlgae());
    }

    m_iTickCount++;
  }
  
  // This runs on teleop init.
  public void teleopInit() {
    
  }

  // This gets called every teleop periodic tick
  public void teleopPeriodic()
  {
    if (m_elevatorSubsystem.isPresent()) {
      // Getting elevator stuff.
      ElevatorSubsystem elevatorSubsystem = m_elevatorSubsystem.get();
      if (m_manualSpeedCommand == null) {
        m_manualSpeedCommand = new ElevatorManualSpeed(elevatorSubsystem);
      }

      // Looks for the bottom (180) degrees +- 45 (135 and 225)
      int pov = m_operatorController.getPOV();
      if (pov >= 135 && pov <= 225) {
        if (m_manualSpeedCommand.isScheduled() == false) {
          m_manualSpeedCommand.schedule();
        }
      } else {
        if (m_manualSpeedCommand.isScheduled() == true) {
          elevatorSubsystem.zeroPosition();
          m_manualSpeedCommand.cancel();
        }
      }
    }
  }
}
