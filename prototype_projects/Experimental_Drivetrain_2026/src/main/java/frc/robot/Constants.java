// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutoConstants {
    // Module Config Stuff
    public static final double kMaxDriveVelocityMPS = 40.0; // This is what SysID is saying i dont think its going 90 mph
    public static final double kWheelCOF = 1.0;
    public static final int kNumMotors = 4;
    public static final double kDriveCurrentLimit = 10.0;

    public static final DCMotor kDriveMotor = DCMotor.getKrakenX60(kNumMotors);
    public static final ModuleConfig kMoudleConfig = new ModuleConfig(
      DrivetrainConstants.kWheelRadiusMeters, 
      kMaxDriveVelocityMPS, 
      kWheelCOF, 
      kDriveMotor, 
      kDriveCurrentLimit, 
      kNumMotors
    );

    // Robot Config Stuff
    // TODO: Run SYS ID and fill in!
    public static final double kMassKG = 15.0;
    public static final double kMOI = kMassKG * (DrivetrainConstants.trackWidthMeters/2) * (DrivetrainConstants.kA_angular / DrivetrainConstants.kA_linear); // Moment of Intertia

    public static final RobotConfig kRobotConfig = new RobotConfig(kMassKG, kMOI, kMoudleConfig, DrivetrainConstants.trackWidthMeters);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.02; // Exclusive
  }

  public static class DrivetrainConstants {
    public static final int kLeftMotorCANID = 10;
    public static final int kOptionalLeftMotorCANID = 11;

    public static final int kRightMotorCANID = 20;
    public static final int kOptionalRightMotorCANID = 21;

    public static final double kTurnDivider = 2;
    public static final double kSpeedDivider = 2.5;

    // These numbers came from the ctre example then tweaked
    public static final double kS = 0.1; // A velocity target of 1 rps results in xV output
    public static final double kV = 0.12; // Add x V output to overcome static friction
    public static final double kP = 0.11; // An error of 1 rotation results in x V output
    public static final double kI = 0.0;
    public static final double kD = 0.0; // A velocity of 1 rps results in x V output
    public static final double kA_linear = 0.01; // Voltage needed to induce a given accel. in the motor shaft
    public static final double kA_angular = 0.01; // TODO: We need to measure this!
    public static final double kPeakVoltage = 8.0;

    public static final double kMaxVelocityMPS = 3.0; // 6 mps is the max of the motors during zero load.
    public static final double kMaxAccelerationMPS2 = 5.0; // M/S^2

    public static final double kMotionMagicAcceleration = 100.0; // Higher number --> Faster (50.0 = ~1s to max)
    public static final double kMotionMagicJerk = 4000.0;

    // For Auto Potentially
    public static boolean kLeftPositiveMovesForward = true;
    public static boolean kRightPositiveMovesForward = true;

    // Physical measurements related to the drivetrain.
    public static final double kDrivetrainGearRatio = 0.2;
    public static final double kWheelRadiusMeters = (4.0 / 2.0) * 0.0254; // Four Inch Wheels
    public static final double kWheelCircumference = 2 * Math.PI * DrivetrainConstants.kWheelRadiusMeters;

    // SmartDashboard update frequency for drive subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 5;

    // The track width in meters.
    public static final double trackWidthMeters = 29.0 * 0.0254; 
  }
}
