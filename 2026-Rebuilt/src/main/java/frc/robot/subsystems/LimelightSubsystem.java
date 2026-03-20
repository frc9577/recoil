// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utils.Pigeon;

public class LimelightSubsystem extends SubsystemBase {
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final Pigeon m_pigeon;

  /** Creates a new PositionSubsystem. */
  public LimelightSubsystem(DifferentialDrivePoseEstimator poseEstimator, Pigeon pigeon) {
    m_poseEstimator = poseEstimator;
    m_pigeon = pigeon;
  }

  // This is copy and pasted from limelight's documentation for testing.
  private void updateOdometry() {
    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");;

    /// Invalidate the vision measurements if some parameters are not met \\\
    if (poseEst == null) { 
      return;
    }

    // If detected tags are not enough
    SmartDashboard.putNumber("Tag Count", poseEst.tagCount);
    if(
      (poseEst.isMegaTag2 == true && poseEst.tagCount < 1) 
      || (poseEst.isMegaTag2 == false && poseEst.tagCount < 2 )
    ) {
      return;
    }

    // Make sure that the estimated position is within field bounds.
    Pose2d estimatedPose = poseEst.pose;
    if (
      estimatedPose.getX() < 0.0 || estimatedPose.getX() > FieldConstants.kFieldWidth 
      || estimatedPose.getY() < 0.0 || estimatedPose.getY() > FieldConstants.kFieldLength
    ) {
      return;
    }

    // Init the mt2 process once a apriltag is first seen via mt1 and a good rotation is set.
    if (poseEst.tagCount > 1) {
      return;
    }

    /// Actually do the vision measurements \\\

    // A Standard Deviation, in the form of [x, y, theta]ᵀ in meters and radians.
    // TODO: Look into scaling these based off of the ambuguity and size of the tags.
    Vector<N3> errorVec = VecBuilder.fill(.7,.7,9999999);

    m_poseEstimator.setVisionMeasurementStdDevs(errorVec);
    m_poseEstimator.addVisionMeasurement(
      poseEst.pose,
      poseEst.timestampSeconds
    );
  }

  @Override
  public void periodic() {
    updateOdometry();
  }
}
