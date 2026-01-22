package frc.robot.factorys;

import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Optional;

public class DriveSubsystemFactory {

  public DriveSubsystemFactory() {}

  public Optional<DriveSubsystem> construct(
    DifferentialDrivePoseEstimator poseEstimator,
    DifferentialDriveKinematics kinematics,
    AHRS gyro,
    Optional<TalonFX> rightLead,
    Optional<TalonFX> leftLead,
    Optional<TalonFX> rightFollower,
    Optional<TalonFX> leftFollower
  ) {
    if (rightLead.isEmpty() || leftLead.isEmpty()) {
      return Optional.empty();
    }

    DriveSubsystem driveSubsystem = new DriveSubsystem(
      poseEstimator,
      kinematics,
      gyro,
      rightLead.get(),
      leftLead.get()
    );

    if (rightFollower.isPresent() && leftFollower.isPresent()) {
      driveSubsystem.setFollowers(rightFollower.get(), leftFollower.get());
    }

    return Optional.of(driveSubsystem);
  }
}
