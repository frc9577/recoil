// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.IMUData;
import frc.robot.utils.LimitedQueue;

public class LimelightSubsystem extends SubsystemBase {
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final AHRS m_gyro;

  private final LimitedQueue<Double> m_YawQue = new LimitedQueue<Double>(5);

  /** Creates a new PositionSubsystem. */
  public LimelightSubsystem(DifferentialDrivePoseEstimator poseEstimator, AHRS gyro) {
    m_poseEstimator = poseEstimator;
    m_gyro = gyro;

    m_gyro.enableLogging(false);
  }

  public double getRobotYaw() {  
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
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (mt2 == null) { 
      return;
    }

    SmartDashboard.putNumber("mt2 Tag Count", mt2.tagCount);
    
    // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    if(Math.abs(m_gyro.getRate()) > 720)
    {
      doRejectUpdate = true;
    }

    if (Math.abs(m_gyro.getPitch()) > 5) {
      doRejectUpdate = true;
    }

    if(mt2.tagCount == 0)
    {
      doRejectUpdate = true;
    }

    if(!doRejectUpdate)
    {
      // A Static Standard Deviation, in the form of [x, y, theta]ᵀ in meters and radians.
      Vector<N3> errorVec;

      // When the robot is disabled, trust the gyro from the limelight fully.
      // Otherwise, in all othr modes, trust the gyro yaw.
      if (DriverStation.isDisabled()) {
        errorVec = VecBuilder.fill(.7,.7,0);
      } else {
        errorVec = VecBuilder.fill(.7,.7,9999999);
      }
      m_poseEstimator.setVisionMeasurementStdDevs(errorVec);

      m_poseEstimator.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds
      );
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler
    IMUData limelightData = LimelightHelpers.getIMUData("limelight");
    m_YawQue.add(limelightData.robotYaw);

    updateOdometry();
  }
}
