package frc.robot.commands;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class RotateToRotation2D extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final double m_maxSpeed;
  private Rotation2d m_targetRotation;

  // Bounds are [0, 1) in interval notation. This effects the steepness of
  // the curve that is used to slow the robot as it approches the target.
  private double speedExp = 0.98;

  /**
   * Creates a new RotateToRotation2D.
   *
   * @param driveSubsystem The driveSubsystem for the robot.
   * @param poseEstimator The poseEstimator of the robot.
   * @param targetRotation The rotation you want the robot to go to.
   * @param maxSpeed The max speed the robot is allowed to go in the rotation.
   */
  public RotateToRotation2D(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, Rotation2d targetRotation, double maxSpeed) 
  {
    m_driveSubsystem = driveSubsystem;
    m_poseEstimator = poseEstimator;
    m_targetRotation = targetRotation;
    m_maxSpeed = maxSpeed;

    SmartDashboard.putNumber("Target Rotation", m_targetRotation.getDegrees());

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d currentRotation = m_poseEstimator.getEstimatedPosition().getRotation();
    Rotation2d changeInRotation = m_targetRotation.minus(currentRotation);

    double targetDiff = Math.abs(changeInRotation.getDegrees());

    double speed = ((Math.pow(speedExp, targetDiff)-1)/(Math.pow(speedExp, 180)-1));
    speed *= m_maxSpeed;
    speed = Math.max(speed, 0.3);
    SmartDashboard.putNumber("Rotation Speed", speed);

    if (changeInRotation.getDegrees() <= 0) {
      m_driveSubsystem.setDifferentialSpeeds(speed, -speed);
    } else {
      m_driveSubsystem.setDifferentialSpeeds(-speed, speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended Rotation fix!");
    m_driveSubsystem.setDifferentialSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Rotation2d currentRotation = m_poseEstimator.getEstimatedPosition().getRotation();
    Rotation2d diff = m_targetRotation.minus(currentRotation);

    double currentDiffAbs = Math.abs(diff.getDegrees());
    SmartDashboard.putNumber("Target Angle Diff Abs", currentDiffAbs);

    if (currentDiffAbs <= 0.5) {
      return true;
    } else {
      return false;
    }
  }
}
