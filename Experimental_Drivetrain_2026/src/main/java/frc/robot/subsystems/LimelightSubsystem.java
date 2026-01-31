// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final AHRS m_gyro;

  /** Creates a new PositionSubsystem. */
  public LimelightSubsystem(DifferentialDrivePoseEstimator poseEstimator, AHRS gyro) {
    m_poseEstimator = poseEstimator;
    m_gyro = gyro;

    LimelightHelpers.SetIMUMode("limelight", 1);
  }

  // This is copy and pasted from limelight's documentation for testing.
  public void updateOdometry() {
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (mt2 == null) { 
      return;
    }
    
    // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    if(Math.abs(m_gyro.getRate()) > 720)
    {
      doRejectUpdate = true;
    }

    if(mt2.tagCount == 0)
    {
      doRejectUpdate = true;
    }

    if(!doRejectUpdate)
    {
      // A Static Standard Deviation of .7 meters for x & y, it does not trust the theta. (Taken from example)
      Vector<N3> errorVec = VecBuilder.fill(.7,.7,9999999);
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
    updateOdometry();
  }
}
