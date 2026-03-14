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
import frc.robot.LimelightHelpers.IMUData;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimitedQueue;
import frc.robot.utils.Pigeon;

public class LimelightSubsystem extends SubsystemBase {
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final Pigeon m_pigeon;

  private final LimitedQueue<Double> m_YawQue = new LimitedQueue<Double>(5);
  private boolean firstApriltagSeen = false;

  /** Creates a new PositionSubsystem. */
  public LimelightSubsystem(DifferentialDrivePoseEstimator poseEstimator, Pigeon pigeon) {
    m_poseEstimator = poseEstimator;
    m_pigeon = pigeon;
  }

  public double getRobotYaw() {  
    if (firstApriltagSeen == false) {
      System.out.println("No apriltag seen! returning 0!");
      return 0.0;
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

  // This is copy and pasted from limelight's documentation for testing.
  private void updateOdometry() {
    PoseEstimate poseEst;
    if (firstApriltagSeen == true) {
      LimelightHelpers.SetRobotOrientation("limelight", m_pigeon.getYaw(), 0, m_pigeon.getPitch(), 0, m_pigeon.getRoll(), 0);
      poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    } else {
      poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }

    if (poseEst == null) { 
      return;
    }

    /// Invalidate the vision measurements if some parameters are not met \\\

    // If theirs no detected tags
    SmartDashboard.putNumber("Tag Count", poseEst.tagCount);
    if(poseEst.tagCount == 0)
    {
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
    if (firstApriltagSeen == false && poseEst.tagCount >= 2 && poseEst.isMegaTag2 == false) {
      firstApriltagSeen = true;
    }

    /// Actually do the vision measurements \\\

    // A Standard Deviation, in the form of [x, y, theta]ᵀ in meters and radians.
    Vector<N3> errorVec;

    // Only trust the yaw from the guess if its not via mt2.
    // TODO: Look into scaling these based off of the ambuguity and size of the tags.
    if (poseEst.isMegaTag2 == false) {
      errorVec = VecBuilder.fill(.7,.7,0);
    } else {
      errorVec = VecBuilder.fill(.7,.7,9999999);
    }

    m_poseEstimator.setVisionMeasurementStdDevs(errorVec);
    m_poseEstimator.addVisionMeasurement(
      poseEst.pose,
      poseEst.timestampSeconds
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler
    if (firstApriltagSeen == true) {
      IMUData limelightData = LimelightHelpers.getIMUData("limelight");
      m_YawQue.add(limelightData.robotYaw);
    } 
    //System.out.println(m_YawQue.toString());

    updateOdometry();
  }
}
