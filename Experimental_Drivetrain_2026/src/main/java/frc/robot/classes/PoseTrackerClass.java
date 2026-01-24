package frc.robot.classes;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

// This is the class that manages all of the pose tracking for the robot.
public class PoseTrackerClass {
    private final DifferentialDrivePoseEstimator m_drivetrainPoseEstimator;
    private final DifferentialDrivePoseEstimator m_limelightPoseEstimator;

    private double m_drivetrainError = 0.0;
    private double m_limelightError = 0.0;
    private double m_totalError = 0.0;

    /** Creates a new PositionSubsystem. */
    public PoseTrackerClass(DifferentialDrivePoseEstimator drivetrainPoseEstimator, DifferentialDrivePoseEstimator limelightPoseEstimator) {
        m_drivetrainPoseEstimator = drivetrainPoseEstimator;
        m_limelightPoseEstimator = limelightPoseEstimator;
    }

    // This makes the most updated pose2d and returns it.
    // TODO: make it actually do the merging n stuf
    public Pose2d getPose(){
        return new Pose2d();
    }

    public void periodic() {
        // This method will be called once per scheduler 

    }
}
