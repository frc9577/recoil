package frc.robot.commands.autoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.PathUtils;

public class OnTheFlyToTarget extends Command {
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private Pose2d m_targetPose;
  private final PathConstraints m_constraints;
  private final Boolean m_reversed;
  private final double m_endVelocity;

  /**
   * This creates and schedules a path on the fly to a given target point.
   *
   * @param poseEstimator The pose estimator to get the location of the robot.
   * @param targetPose The pose the robot will make a point to go to.
   * @param constraints The constraints the robot will follow while following the path.
   * @param endVelocity The velocity it should be at the end of the path.
   * @param reversed If the robot should run the path in reversed (backward)
   */
  public OnTheFlyToTarget(DifferentialDrivePoseEstimator poseEstimator, Pose2d targetPose, PathConstraints constraints, Double endVelocity, Boolean reversed) 
  {
    m_poseEstimator = poseEstimator;
    m_targetPose = targetPose;
    m_constraints = constraints;
    m_endVelocity = endVelocity;
    m_reversed = reversed;
  }

  /**
   * This creates and schedules a path on the fly to a given target point.
   *
   * @param poseEstimator The pose estimator to get the location of the robot.
   * @param targetPose The pose the robot will make a point to go to.
   * @param constraints The constraints the robot will follow while following the path.
   * @param endVelocity The velocity it should be at the end of the path.
   */
  public OnTheFlyToTarget(DifferentialDrivePoseEstimator poseEstimator, Pose2d targetPose, PathConstraints constraints, Double endVelocity) 
  {
    this(poseEstimator, targetPose, constraints, endVelocity, false);
  }

  /**
   * This creates and schedules a path on the fly to a given target point.
   *
   * @param poseEstimator The pose estimator to get the location of the robot.
   * @param targetPose The pose the robot will make a point to go to.
   * @param constraints The constraints the robot will follow while following the path.
   * @param reversed If the robot should run the path in reversed (backward)
   */
  public OnTheFlyToTarget(DifferentialDrivePoseEstimator poseEstimator, Pose2d targetPose, PathConstraints constraints, Boolean reversed) 
  {
    this(poseEstimator, targetPose, constraints, 0.0, reversed);
  }

  /**
   * This creates and schedules a path on the fly to a given target point.
   * 
   * @param poseEstimator The pose estimator to get the location of the robot.
   * @param targetPose The pose the robot will make a point to go to.
   * @param constraints The constraints the robot will follow while following the path.
   */
  public OnTheFlyToTarget(DifferentialDrivePoseEstimator poseEstimator, Pose2d targetPose, PathConstraints constraints) 
  {
    this(poseEstimator, targetPose, constraints, 0.0, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
    PathPlannerPath path = PathUtils.OnTheFlyToTarget(currentPose, m_targetPose, m_constraints, m_endVelocity, m_reversed);
    
    // Create the command version of the path and return it
    Command followPath = AutoBuilder.followPath(path);
    CommandScheduler.getInstance().schedule(followPath);
  }
}
