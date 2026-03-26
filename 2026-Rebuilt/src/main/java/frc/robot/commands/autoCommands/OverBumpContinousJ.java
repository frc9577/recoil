package frc.robot.commands.autoCommands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimAtHub;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.PathUtils;

/** An example command that uses an example subsystem. */
public class OverBumpContinousJ extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final BooleanSupplier m_isRed;

  /**
   * Creates a new OverBumpContinousJ.
   *
   * @param poseEstimator The pose estimator to get the location of the robot.
   * @param targetPose The pose the robot will make a point to go to.
   * @param constraints The constraints the robot will follow while following the path. 
   */
  public OverBumpContinousJ(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed) 
  {
    m_driveSubsystem = driveSubsystem;
    m_poseEstimator = poseEstimator;
    m_isRed = isRed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Deadreckon Over the bump
    Command deadreckonFoward1 = new DeadreckonDistance(m_driveSubsystem, 2.5, 2.0);
    Command deadreckonFoward2 = new DeadreckonDistance(m_driveSubsystem, 2.5, 2.0);

    // Create paths and scheudle other commands
    Command pickupScheudle = new InstantCommand(() -> {
      Pose2d currentPose = m_poseEstimator.getEstimatedPosition();

      try {
        PathPlannerPath ContJ = PathPlannerPath.fromPathFile("ContinuousJ");
        PathPlannerPath editedPath = PathUtils.modifyPath(ContJ, new Pair<Integer, Pose2d>(0, currentPose));
        //PathPlannerPath editedPath = PathUtils.appendToPath(ContJ, 0, currentPose);
        Command pathCommand = AutoBuilder.followPath(editedPath);

        // Create & Scheudle the command group
        CommandScheduler.getInstance().schedule(
          pathCommand
          .andThen(deadreckonFoward2)
          .andThen(new AimAtHub(m_driveSubsystem, m_poseEstimator, 2.0, m_isRed))
          .andThen(new DeadreckonDistance(m_driveSubsystem, 0.75, -1.0))
          .andThen(new AimAtHub(m_driveSubsystem, m_poseEstimator, 2.0, m_isRed))
        );
      } catch (Exception e) {
        DriverStation.reportError(e.toString(), true);
      }
    });
    
    // Create & Schedule the command group.
    //SequentialCommandGroup initialSequential = deadreckonFoward1.andThen(pickupScheudle);
    CommandScheduler.getInstance().schedule(pickupScheudle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The command automatically ends
    return true;
  }
}
