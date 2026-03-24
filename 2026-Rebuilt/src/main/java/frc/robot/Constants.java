// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//
// CANIDs and other hardware resource IDs and channels are defined in
// the robot specification document at https://docs.google.com/document/d/1A_Mh49vCdzeZFZrrAAlE3kgAf9tdAOItZ7-rJoq3Ufs/edit?usp=sharing
//
// Current CAN IDs:
//
//   01 - Pneumatics hub
//   02 - Pigeon 2
//   10 - Drive left primary
//   11 - Drive left follower
//   20 - Drive right primary
//   21 - Drive right follower
//   30 - Intake roller
//   40 - Launcher flywheel primary motor  
//   41 - Launcher flywheel secondary motor
//   42 - Launcher lift belt motor
//   50 - Indexer motor
//   51 - Bulk move belt motor
//
// Current DIO Channels:
//
// 0 - Upper fuel sensor (launcher subsystem)
// 1 - 
//
// Current Solenoid Channels:
//
// 0 - Intake double channel 1
// 1 - Intake double channel 2
// 2 - Climb L1 Left
// 3 - Climb L1 Right
 
package frc.robot;

import java.util.Map;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class FieldConstants {
    public static final Pose2d kBlueHubCenter = new Pose2d(4.607, 4.035, new Rotation2d());
    public static final Pose2d kRedHubCenter = new Pose2d(11.9284, 4.035, new Rotation2d());

    public static final double kFieldWidth = 16.540988; // meters, x axis.
    public static final double kFieldLength = 8.069326; // meters, y axis.
  }

  public static class RobotConstants {
    public static final int kPigeon2CANID = 2;
    public static final boolean kDoPigeonWarn = true;
    public static final double kPigeonYawOffset = -90.00460052490234; // degrees (-360, 360)
    public static final double kPigeonPitchOffset = 1.5642017126083374; // degrees (-360, 360)
    public static final double kPigeonRollOffset = 89.49134826660156; // degrees (-360, 360)

    public static enum kStartingNames {
      DEPOT_SIDE,
      HUB,
      OUTPOST_SIDE
    }

    public static final Map<kStartingNames, Map<Boolean, Pose2d>> kStartingPositions = Map.ofEntries(
      Map.entry(kStartingNames.DEPOT_SIDE, Map.ofEntries(
          Map.entry(true, new Pose2d(new Translation2d(12.956, 2.884), Rotation2d.k180deg)), // red
          Map.entry(false, new Pose2d(new Translation2d(3.607, 5.186), Rotation2d.kZero)) // blue
      )),
      Map.entry(kStartingNames.HUB, Map.ofEntries(
          Map.entry(true, new Pose2d(new Translation2d(12.945, 4.0), Rotation2d.k180deg)), // red
          Map.entry(false, new Pose2d(new Translation2d(3.561, 4.0), Rotation2d.kZero)) // blue
      )),
      Map.entry(kStartingNames.OUTPOST_SIDE, Map.ofEntries(
          Map.entry(true, new Pose2d(new Translation2d(12.979, 5.163), Rotation2d.k180deg)), // red
          Map.entry(false, new Pose2d(new Translation2d(3.6, 2.9), Rotation2d.kZero)) // blue
      ))
    );
  }

  public static class AutoConstants {
    public static final double kMaxDriveVelocityMPS = 40.0; // true max speed of the robot mps
    public static final double kWheelCOF = 1.0; // no data for this, bsed it

    public static final double kMassKG = 15.0;

    // Uncommenting this causes the weird x y rotion error.
    //public static final double kMOI = (1/12)  * kMassKG * ((DrivetrainConstants.kLengthMeters*DrivetrainConstants.kLengthMeters) + (DrivetrainConstants.kWidthMeters*DrivetrainConstants.kWidthMeters)); // Moment of Intertia
    public static final double kMOI = kMassKG * (DrivetrainConstants.kTrackWidthMeters/2) * (DrivetrainConstants.kA_angular / DrivetrainConstants.kA_linear); // Moment of Intertia

    public static final double kDriveCurrentLimit = 10.0; // amps

    // These values may change per bot, we might want to populate these on init w/ accurate data
    public static final int kNumMotors = 4; // # of motors in a gearbox

    public static final DCMotor kDriveMotor = DCMotor.getKrakenX60(kNumMotors);
    public static final ModuleConfig kModuleConfig = new ModuleConfig(
      DrivetrainConstants.kWheelRadiusMeters, 
      kMaxDriveVelocityMPS, 
      kWheelCOF, 
      kDriveMotor, 
      kDriveCurrentLimit, 
      kNumMotors
    );

    public static final RobotConfig kRobotConfig = new RobotConfig(kMassKG, kMOI, kModuleConfig, DrivetrainConstants.kTrackWidthMeters);
    // populate on init possibility end here
  }

  public static class PneumaticsConstants {
    public static final PneumaticsModuleType kHubType = PneumaticsModuleType.REVPH;
    public static final int kPneumaticsHubCANID = 1;
    public static final double kMinPneumaticsPressure = 80.0;
    public static final double kMaxPneumaticsPressure = 115.0;

    public static final int kTicksPerUpdate = 5;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.02; // Exclusive

    public static final int kOperatorControllerPort = 1;
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
    public static final double kBumpersMeters = 0.0889;
    public static final double kWidthMeters = 0.8636-kBumpersMeters; // width w/ bumpers - bumpers
    public static final double kLengthMeters = 0.8636-kBumpersMeters;
    public static final double kTrackWidthMeters = kLengthMeters;//29.0 * 0.0254;//0.74; 

    // SmartDashboard update frequency for drive subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 5;
  }

  public static class IntakeConstants {
    public static int kIntakeMotorCANID = 30;

    // Raw intake motor speed in range [-1.0,1.0]
    public static double kIntakeMotorSpeed = 0.6;

    // Solenoid states required to extend and retract the intake mechanism.
    public static int     kIntakeSolenoidForward = 15;
    public static int     kIntakeSolenoidReverse = 13;
    public static DoubleSolenoid.Value kIntakeExtend   = DoubleSolenoid.Value.kForward;
    public static DoubleSolenoid.Value kIntakeRetract  = DoubleSolenoid.Value.kReverse;
  }

  public static class LauncherConstants {
    public static final int kLauncherFlywheelMotor1CANID = 40; 
    public static final int kLauncherFlywheelMotor2CANID = 41;
    public static final int kLauncherLiftMotorCANID      = 42;

    // The speed, in range [-1.0, 1.0], to run the lift motor when started.
    public static final double kLiftMotorSpeed = 1.0;

    // Beam break sensor to detect fuel at the top of the lift.
    public static final int kUpperFuelSensorChannel = 0;
    public static final boolean kUpperFuelSensorIsEmpty = true;

    // These numbers came from the ctre example then tweaked
    public static final double kS = 0.1; // A velocity target of 1 rps results in xV output
    public static final double kV = 0.12; // Add x V output to overcome static friction
    public static final double kP = 0.11; // An error of 1 rotation results in x V output
    public static final double kI = 0.0;
    public static final double kD = 0.0; // A velocity of 1 rps results in x V output
    public static final double kA_linear = 0.01; // Voltage needed to induce a given accel. in the motor shaft
    public static final double kA_angular = 0.01; // TODO: We need to measure this!
    public static final double kPeakVoltage = 8.0;

    public static final double kMaxVelocityRPS = 6000.0/60.0;
    public static final double kMaxAccelerationRPS2 = 50.0;

    public static final double kMotionMagicAcceleration = 50.0; // Higher number --> Faster (50.0 = ~1s to max)
    public static final double kMotionMagicJerk = 4000.0;

    // Set to false if both flywheel motors drive in the same direction, false if they
    // run in opposite directions.
    public static final boolean kMotorsDriveInOppositeDirections = true;

    // Set to false if we drive the lead launcher flywheel motor clockwise to operate
    // correctly, or true to drive it counterclockwise.
    public static final boolean kLauncherMotorForwardIsCCW = false;

    // Frequency at which we send current launcher speed back to the driver station.
    public static final int kTicksPerUpdate = 10;
    public static final int kTicksPerDistanceUpdate = 12;

    public static final double kFixedTestSpeed = 3000.0;
    public static final double kFlywheelToleranceRPM = 100.0;
  }

  public static class IndexerBulkConstants {
    public static int kIndexerMotorCANID  = 50;
    public static int kBulkMoveMotorCANID = 51;

    // Raw motor speeds in range [-1.0,1.0]
    public static final double kBulkMoveMotorSpeed = 0.5;
    public static final double kIndexerMotorSpeed = 0.3;
  }

  public static class ClimbL1Constants {
    public static int kLeftSolenoidChannel  = 1;
    public static int kRightSolenoidChannel = 2;

    public static boolean kClimbL1Raise  = true;
    public static boolean kClimbL1Lower  = !kClimbL1Raise;
  }
}
