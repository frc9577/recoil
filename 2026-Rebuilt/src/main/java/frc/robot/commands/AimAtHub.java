package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.HubUtils;
/** An example command that uses an example subsystem. */
public class AimAtHub extends RotateToRotation2D {
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final BooleanSupplier m_isRed;

  /**
   * Creates a new AimAtHub.
   *
   * @param driveSubsystem The driveSubsystem for the robot.
   * @param poseEstimator The poseEstimator of the robot.
   * @param maxSpeed The max speed the robot is allowed to go in the rotation.
   * @param isRed Does the robot need to point towards the red or blue goal, default is blue.
   */
  public AimAtHub(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, double maxSpeed, BooleanSupplier isRed) 
  {
    super(driveSubsystem, poseEstimator, Rotation2d.kZero, maxSpeed);

    m_poseEstimator = poseEstimator;
    m_isRed = isRed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotation2d targetRotation = HubUtils.getRobotToHubAngle(m_poseEstimator, m_isRed);
    super.setTargetRotation(targetRotation);
    super.initialize();
  }
}
