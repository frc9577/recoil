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
import frc.robot.commands.RotateToRotation2D;
import frc.robot.commands.util.AutoFromList;
import frc.robot.commands.util.AutoFromList.firstPathType;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class BackupAndClimb extends Command {
  private final ArrayList<Object> m_preloadPaths = new ArrayList<Object>(Arrays.asList(
    "LineupToTower"
  ));

  // Pass-ins
  private final DriveSubsystem m_DriveSubsystem;
  private final DifferentialDrivePoseEstimator m_PoseEstimator;
  private final BooleanSupplier m_isRed;
  private final PathConstraints m_constraints;

  /**
   * Creates a new DSA_BackupTest.
   */
  public BackupAndClimb(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed, PathConstraints constraints) {
    m_DriveSubsystem = driveSubsystem;
    m_PoseEstimator = poseEstimator;
    m_isRed = isRed;
    m_constraints = constraints;

    // warm-up for quick load later on
    for (Object item : m_preloadPaths) {
        if (item instanceof String) {
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
    Rotation2d targetRot;
    if (m_isRed.getAsBoolean() == true) {
      targetRot = Rotation2d.kZero;
    } else {
      targetRot = Rotation2d.k180deg;
    }

    // Init the new sequence
    ArrayList<Object> sequence = new ArrayList<Object>(Arrays.asList(
      new DeadreckonDistance(m_DriveSubsystem, 1.0, -1),
      "LineupToTower",
      new RotateToRotation2D(m_DriveSubsystem, m_PoseEstimator, targetRot, 1.0)
    ));

    // Run the command
    Command auto = new AutoFromList(sequence, m_constraints, m_DriveSubsystem, m_PoseEstimator, firstPathType.GO_TO_FIRST_PATH, false);
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
