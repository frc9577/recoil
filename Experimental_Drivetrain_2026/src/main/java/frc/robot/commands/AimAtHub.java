package frc.robot.commands;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class AimAtHub extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final double m_maxSpeed;
  private final boolean m_isRed;

  private final Pose2d m_BlueCenter = new Pose2d(4.607, 4.035, new Rotation2d());
  private final Pose2d m_RedCenter = new Pose2d();

  /**
   * Creates a new AimAtHub.
   *
   * @param driveSubsystem The driveSubsystem for the robot.
   * @param poseEstimator The poseEstimator of the robot.
   * @param maxSpeed The max speed the robot is allowed to go in the rotation.
   * @param isRed Does the robot need to point tword the red or blue goal, default is blue.
   */
  public AimAtHub(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, double maxSpeed, boolean isRed) 
  {
    m_driveSubsystem = driveSubsystem;
    m_poseEstimator = poseEstimator;
    m_maxSpeed = maxSpeed;
    m_isRed = isRed;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d targetPose;
    if (m_isRed) {
      targetPose = m_RedCenter;
    } else {
      targetPose = m_BlueCenter;
    }

    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
    double dY = targetPose.getY() - currentPose.getY();
    double dX = targetPose.getX() - currentPose.getX();

    Rotation2d targetRotation = new Rotation2d(Math.atan2(dY, dX));
    RotateToRotation2D aimCommand = new RotateToRotation2D(
      m_driveSubsystem, 
      m_poseEstimator, 
      targetRotation, 
      m_maxSpeed
    );

    CommandScheduler.getInstance().schedule(aimCommand);
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
    return true;
  }
}
