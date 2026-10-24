// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

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
import frc.robot.utils.LimitedQueue;

public class LimelightSubsystem extends SubsystemBase {
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final LimitedQueue<Double> m_YawQue = new LimitedQueue<Double>(5);
  private boolean m_allowJumps = true;

  /** Creates a new PositionSubsystem. */
  public LimelightSubsystem(DifferentialDrivePoseEstimator poseEstimator) {
    m_poseEstimator = poseEstimator;
  }

  public Double getRobotYaw() {  
    if (m_YawQue.isEmpty()) {
      return null;
    }

    Double[] doubleObjects = m_YawQue.toArray(new Double[0]);
    int queSize = doubleObjects.length;

    double[] primitiveDoubles = new double[queSize];
    for (int i = 0; i < queSize; i++) {
        primitiveDoubles[i] = doubleObjects[i].doubleValue();
    }

    SimpleMatrix samplesMatrix = new SimpleMatrix(primitiveDoubles);

    double[] weightsArray = new double[queSize];
    Arrays.fill(weightsArray, 1.0);

    SimpleMatrix weightsMatrix = new SimpleMatrix(1, queSize, true, weightsArray);
    
    double avrg = samplesMatrix.dot(weightsMatrix)/weightsMatrix.elementSum();
    return avrg;
  }

  private void updateOdometry() {
    PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    /// Invalidate the vision measurements if some parameters are not met \\\
    if (poseEst == null) { 
      return;
    }

    Pose2d estimatedPose = poseEst.pose;
    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();

    // If detected tags are not enough
    SmartDashboard.putNumber("Tag Count", poseEst.tagCount);
    if(poseEst.tagCount < 2 || poseEst.rawFiducials.length <= 0) {
      return;
    }

    // Make sure that the estimated position is within field bounds.
    if (
      estimatedPose.getX() < 0.0 || estimatedPose.getX() > FieldConstants.kFieldWidth 
      || estimatedPose.getY() < 0.0 || estimatedPose.getY() > FieldConstants.kFieldLength
    ) {
      return;
    }

    double distanceDiff = currentPose.getTranslation().getDistance(estimatedPose.getTranslation());
    if (m_allowJumps == false && distanceDiff > 2.0) {
      return;
    }

    // Make sure that the abiguity of the primary target is not to high
    Double bestTargetAmbiguity = poseEst.rawFiducials[0].ambiguity;
    if (bestTargetAmbiguity > 0.2) {
      return;
    }

    /// Actually do the vision measurements \\\

    // A Standard Deviation, in the form of [x, y, theta]ᵀ in meters and radians.
    double stdDev = 0.5 + bestTargetAmbiguity;
    Vector<N3> errorVec = VecBuilder.fill(stdDev, stdDev,9999999);

    // Trusting the yaw when disabled to get our robot orientation.
    // if (DriverStation.isDisabled()) {
    //   errorVec = VecBuilder.fill(stdDev, stdDev,0);
    // }

    // Add the measurements to the pose estimator
    m_poseEstimator.setVisionMeasurementStdDevs(errorVec);
    m_poseEstimator.addVisionMeasurement(
      estimatedPose,
      poseEst.timestampSeconds
    );

    m_YawQue.add(estimatedPose.getRotation().getDegrees());
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public void setAllowJumps(boolean allowJumps) {
    m_allowJumps = allowJumps;
  }

  public void resetYawQue() {
    m_YawQue.clear();
  }
}
