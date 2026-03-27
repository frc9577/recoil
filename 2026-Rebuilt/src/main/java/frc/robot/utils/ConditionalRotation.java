package frc.robot.utils;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.RotateToRotation2D;
import frc.robot.subsystems.DriveSubsystem;

public class ConditionalRotation {
    /**
     * This creates a conditional rotation command, where it only rotates if its outside of the expected error.
     * 
     * @param driveSubsystem The drive subsystem.
     * @param poseEstimator The pose estimator.
     * @param targetRotation The desired Rotation.
     * @param error The maximum accepted error in degrees.
     * @param speed The maximum speed the rotation command will go.
    */
    public static Command New(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, Rotation2d targetRotation, double error, double speed) {
        return Commands.either(
            new InstantCommand(), 
            new RotateToRotation2D(
                driveSubsystem, 
                poseEstimator,
                targetRotation,
                speed
            ), 
            () -> {
                Rotation2d currentRot = poseEstimator.getEstimatedPosition().getRotation();
                double diff = Math.abs((targetRotation.minus(currentRot)).getDegrees());

                return diff <= error;
            }
        );
    }
}
