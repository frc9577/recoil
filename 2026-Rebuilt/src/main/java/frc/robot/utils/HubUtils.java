package frc.robot.utils;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;

import java.util.function.BooleanSupplier;

public class HubUtils {
    private static Pose2d getHubPose(BooleanSupplier isRedSup) {
        Boolean isRed = isRedSup.getAsBoolean();
        if (isRed == true) {
            return FieldConstants.kRedHubCenter;
        } else {
            return FieldConstants.kBlueHubCenter;
        }
    }

    // Returns the rotation the robot needs to face, field relative, to be looking at the hub.
    public static Rotation2d getRobotToHubAngle(DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed) {
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        Pose2d hubPose = getHubPose(isRed);

        PoseDiff dPose = new PoseDiff(currentPose, hubPose);

        Rotation2d targetRotation = new Rotation2d(Math.atan2(dPose.y, dPose.x));
        return targetRotation;
    }

    // Returns the distance to your team hub in meters.
    public static Double getHubDistance(DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed) {
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        Pose2d hubPose = getHubPose(isRed);

        PoseDiff length = new PoseDiff(currentPose, hubPose);

        Double lenSq = Math.pow(length.x, 2) + Math.pow(length.y, 2);
        Double len = Math.sqrt(lenSq);

        return len;
    }
}
