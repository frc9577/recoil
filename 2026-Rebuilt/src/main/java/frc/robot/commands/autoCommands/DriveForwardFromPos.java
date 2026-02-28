package frc.robot.commands.autoCommands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class DriveForwardFromPos extends Command {
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private double m_targetDistance = 0.0;

  /**
   * Creates a new DriveForward2mFromPos.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForwardFromPos(DifferentialDrivePoseEstimator poseEstimator, double targetDistance) 
  {
    m_poseEstimator = poseEstimator;
    m_targetDistance = targetDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This is taken and modified from the limelight documentation for
    // path creation on the fly: https://pathplanner.dev/pplib-create-a-path-on-the-fly.html

    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
    Rotation2d currentRotation = currentPose.getRotation();

    double currentX = currentPose.getX();
    double currentY = currentPose.getY();

    // If we think of the direction we want to go as of the hypotonuse of a
    // right trinagle, we can calculate the needed distance to travel on
    // the (x, y) by calculating for them as the legs of the RT.
    double xMod = m_targetDistance * currentRotation.getCos();
    double yMod = m_targetDistance * currentRotation.getSin();

    double newX = currentX + xMod;
    double newY = currentY + yMod;

    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(currentX, currentY, currentRotation),
            new Pose2d(newX, newY, currentRotation)
    );

    PathConstraints constraints = new PathConstraints(
            1.0, 
            1.0, 
            2 * Math.PI,
            4 * Math.PI
    ); // The constraints for this path.

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, currentRotation) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
    
    // Create the command version of the path and schedule it
    Command followPath = AutoBuilder.followPath(path);
    CommandScheduler.getInstance().schedule(followPath);
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
