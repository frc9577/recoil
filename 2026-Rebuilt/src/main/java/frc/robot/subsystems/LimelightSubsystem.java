// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimelightSubsystem extends SubsystemBase {
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  /** Creates a new PositionSubsystem. */
  public LimelightSubsystem(DifferentialDrivePoseEstimator poseEstimator) {
    m_poseEstimator = poseEstimator;
  }

  // This is copy and pasted from limelight's documentation for testing.
  private void updateOdometry() {
    PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    /// Invalidate the vision measurements if some parameters are not met \\\
    if (poseEst == null) { 
      return;
    }

    // If detected tags are not enough
    SmartDashboard.putNumber("Tag Count", poseEst.tagCount);
    if(poseEst.tagCount < 2 || poseEst.rawFiducials.length > 0) {
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

    // Make sure that the abiguity of the primary target is not to high
    Double bestTargetAmbiguity = poseEst.rawFiducials[0].ambiguity;
    if (bestTargetAmbiguity > 0.2) {
      return;
    }

    /// Actually do the vision measurements \\\

    // A Standard Deviation, in the form of [x, y, theta]ᵀ in meters and radians.
    // TODO: Look into scaling these based off of the ambuguity and size of the tags. -- Kinda done (3/20/2025)
    Vector<N3> errorVec;
    double stdDev = 0.5 + bestTargetAmbiguity;

    // Trusting the yaw when disabled to get our robot orientation.
    if (DriverStation.isDisabled()) {
      errorVec = VecBuilder.fill(stdDev, stdDev,0);
    } else {
      errorVec = VecBuilder.fill(stdDev, stdDev,9999999);
    }

    // Add the measurements to the pose estimator
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
