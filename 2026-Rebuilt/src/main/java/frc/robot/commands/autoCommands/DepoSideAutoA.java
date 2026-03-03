package frc.robot.commands.autoCommands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.commands.RotateToRotation2D;
import frc.robot.commands.StartIntakeCommand;
import frc.robot.commands.StartLiftCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.StopLiftCommand;
import frc.robot.commands.util.AutoFromList;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DepoSideAutoA extends Command {
  // The sequance of the command, any "*" are place holders for commands.
  private final ArrayList<Object> m_preloadPaths = new ArrayList<Object>(Arrays.asList(
    "DSA_PickupFwd",
    "DSA_PickupBwd", 
    "DSA_ParkAtWall",
    "DSA_A_Bump"
  ));

  // Pass-ins
  private final DriveSubsystem m_DriveSubsystem;
  private final DifferentialDrivePoseEstimator m_PoseEstimator;
  private final BooleanSupplier m_isRed;
  private final PathConstraints m_constraints;

  /**
   * Creates a new DepoSideAutoA.
   */
  public DepoSideAutoA(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed, PathConstraints constraints) {
    m_DriveSubsystem = driveSubsystem;
    m_PoseEstimator = poseEstimator;
    m_isRed = isRed;
    m_constraints = constraints;
    
    // warm-up for quick load later on
    for (Object item : m_preloadPaths) {
        if (item instanceof String && (String) item != "*") {
          try {
              PathPlannerPath path = PathPlannerPath.fromPathFile((String) item);
              Command command = AutoBuilder.followPath(path);
              new PathPlannerAuto(command);
          } catch (Exception e) {
              System.out.println(e);
          }
        }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get target rotation
    Rotation2d targetRotation;
    if (m_isRed.getAsBoolean() == true) {
      targetRotation = new Rotation2d(0); // on red, aim at blue wall
    } else {
      targetRotation = new Rotation2d(180); // on blue, aim at red wall
    }

    // Init the new sequence
    // TODO: Replace placeholder InstantCommands with real ones.
    ArrayList<Object> sequence = new ArrayList<Object>(Arrays.asList(
      new ExtendIntakeCommand(null), // Extend Intake
      new InstantCommand(), // Drive Over bump fwd
      new InstantCommand(), // Relocate

      // Start Pickup \\
      new StartIntakeCommand(null), // Trigger pickup
      new StartLiftCommand(null), // Trigger Lift
      "DSA_PickupFwd", // ⚠️⚠️ Might have an issue with location being to far from this path ⚠️⚠️

      // End Pickup \\
      new StopIntakeCommand(null), // Disable Pickup
      new StopLiftCommand(null), // Disable Lift

      // Ready shoot & travel \\
      new RotateToRotation2D(m_DriveSubsystem, m_PoseEstimator, targetRotation, 1), // Aim at other wall
      "DSA_PickupBwd", 
      new InstantCommand(), // Spin-up flywheel?? (Might have better location.)
      new InstantCommand(), // Drive over bump bwd
      new InstantCommand(), // Relocate
      "DSA_ParkAtWall", // ⚠️⚠️ Might have an issue with location being to far from this path ⚠️⚠️
      new AimAtHub(m_DriveSubsystem, m_PoseEstimator, 2.0, m_isRed), // Aim at hub

      // Shoot all balls \\
      new InstantCommand(), // Full engage flywheel or wtv
      new StartLiftCommand(null),

      // Ramp down flywheel \\
      new StopLiftCommand(null),
      new InstantCommand(), // Disable flywheel

      // Get ready to pickup when auto ends \\
      "DSA_A_Bump",
      new InstantCommand(), // Drive over bump fwd
      new InstantCommand() // Relocate
    ));

    // Run the command
    Command auto = new AutoFromList(sequence, m_constraints, m_DriveSubsystem, m_PoseEstimator, false, false);
    CommandScheduler.getInstance().schedule(auto);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
