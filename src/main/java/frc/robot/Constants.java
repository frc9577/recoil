// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 //
 // When updating this file, make sure that the following rules apply:
 //
 // 1. CAN IDs must be unique across the whole robot (assuming we only have a single
 // CAN bus which is true this year).
 // 2. Sensor channels must be unique across the whole robot (assuming we're using DIO
 // channels for light break or switch-based sensors).
 // 3. Pneumatics channels must be unique across the whole robot (assuming we're using
 // a single pneumatics hub)

 // Current CAN IDs:
 //   01 - Pneumatics hub
 //   10 - Drive left primary
 //   11 - Drive left follower
 //   20 - Drive right primary
 //   21 - Drive right follower
 //   40 - Coral handler input side motor  
 //   41 - Coral handler output side motor
 //   50 - Algae handler motor
 //   60 - Elevator motor
 //   70 - Coral ramp intake motor
 //
 // Current DIO Channels:
 //
 // 0 - Coral handler front sensor
 // 1 - Coral handler rear sensor
 // 2 - Coral ramp intake sensor
 // 3 - Algae sensor
 // 4 - Elevator bottom sensor
 // 5 -
 // 6 -
 // 7 -
 //

 public final class Constants {
  public static class RobotConstants {
    public static final int kpnuematicsTicksPerUpdate = 5;

    public static final double minPnuematicsPressure = 80.0;
    public static final double maxPnuematicsPressure = 120.0;

    /** Constants that define the settings of the driver camera */
    public static final int kDriverCameraResolutionX = 640;
    public static final int kDriverCameraResolutionY = 360;
    public static final int kDriverCameraFPS         = 10;
  }

  public static class OperatorConstants {
    public static final int kOperatorControllerPort = 0;
    public static final int kDriverControllerPort = 1;

    public static final int kZeroElevator = 3; // Button X
    public static final int kDecreeseElevatorLevel = 1; // Button A
    public static final int kIntakeElevatorPosition = 2; // Button B
    public static final int kIncreaseElevatorLevel = 4; // Button Y

    // In the future when disabling buttons, please comment out the command bind
    // under the configureBindings function in the RobotContrainer. - Owen
    public static final int kCoralIntake = 6; // Right Bumper
    public static final int kCoralOuttake = 5; // Left Bumper
    public static final int kCoralStop = 9; // Left Thumbstick Button

    public static final int kShooterShoot = 5; // Left Bumper

    public static final int kAlageIntake = 8; // START
    public static final int kAlageOutput = 7; // BACK

    // driver controls
    public static final int kEnableReverse = 1; // Button A
    public static final int kDisableReverse = 4; // Button Y
  }

  public static class DrivetrainConstants {
    public static final int kLeftMotorCANID = 10;
    public static final int kOptionalLeftMotorCANID = 11;

    public static final int kRightMotorCANID = 20;
    public static final int kOptionalRightMotorCANID = 21;

    public static final double kTurnDivider = 2;
    public static final double kSpeedDivider = 1.5;

    // Auto PID stuff
    public static final double kV = 0; // Add x V output to overcome static friction
    public static final double kS = 0; // A velocity target of 1 rps results in xV output
    public static final double kP = 0.3; // An error of 1 rotation results in x V output
    public static final double kI = 0.0;
    public static final double kD = 0.1; // A velocity of 1 rps results in x V output
    public static final double PeakVoltage = 10.0;

    public static final int maxVelocity = 30; // rps/s
    public static final int maxAcceleration = 50; // rps

    // For Auto Potentially
    public static boolean kLeftPositiveMovesForward = true;
    public static boolean kRightPositiveMovesForward = true;

    // The distance travelled for a single rotation of the Kraken output shaft.
    public static final double kDrivetrainGearRatio = (8.46/0.478536);

    // SmartDashboard update frequency for drive subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 5;
  }

  public static class AutoConstants {
    public static final double kPassLineSpeed = 0.40;
    public static final double kTurnInnerSpeed = 0.35;
  }

  public static class IntakeConstants {
    public static final int kMotorCANID = 70;

    public static final int kSensorChannel = 2;
    public static final boolean kSensorFalseIsEmpty = false;

    // SmartDashboard update frequency for intake subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 22;
  }

  public static class CoralConstants {
    public static final int kIntakeMotorCANID = 40;
    public static final int kOutputMotorCANID = 41;

    public static final double kMotorIntakeSpeed = 1.0;
    public static final double kMotorOutputSpeed = -1.0;

    // number of ticks between sensor change and motor stop
    public static final int kEndIntakeMaxCount = 6; // ~0.125 second(s)
    public static final int kEndOutputMaxCount = 50; // 1 second(s)

    // Line Break? sensor to detect coral in the middle
    public static final int kSensorChannel = 0;
    public static final boolean kSensorFalseIsEmpty = false;

    // SmartDashboard update frequency for coral subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 10;
  }

  public static class ShooterConstants {
    public static final int kIntakeMotorCANID = 40;
    public static final int kOutputMotorCANID = 41;

    public static final double kMotorIntakeSpeed = 1.0;
    public static final double kMotorOutputSpeed = -1.0;

    // number of ticks between sensor change and motor stop
    public static final int kEndIntakeMaxCount = 6; // ~0.125 second(s)
    public static final int kEndOutputMaxCount = 50; // 1 second(s)

    // Line Break? sensor to detect coral in the middle
    public static final int kSensorChannel = 0;
    public static final boolean kSensorFalseIsEmpty = false;

    // SmartDashboard update frequency for coral subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 10;
  }

  public static class AlgaeConstants {
    public static final int kMotorCANID = 50;
    public static final int kMotorCurrentLimit = 7;

    public static final int kPneumaticsHubCANid = 1;
    public static final int kSolenoidChannel1 = 0;
    public static final int kSolenoidChannel2 = 15;

    public static final int kSensorChannel = 3;
    public static final boolean kSensorFalseIsEmpty = true;

    // number of ticks between sensor change and motor stop
    public static final int kEndIntakeMaxCount = 15; // 0 second(s)
    public static final int kEndOutputMaxCount = 100; // 2 second(s)

    public static final double kIntakeSpeed = 1;
    public static final double kOutputSpeed = -1  ;

    // SmartDashboard update frequency for algae subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 25;
  }

  public static class ElevatorConstants {
    public static final int kMotorCANID = 60;
    public static final double kMotorZeroingSpeed = -0.5;
    public static final boolean kPositiveMovesUp = false;

    public static final double kV = 0;
    public static final double kS = 0;
    public static final double kP = 0.4;// An error of 1 rotation results in 2.4 V output
    public static final double kI = 0.001; // No output for integrated error
    public static final double kD = 0.15; // A velocity of 1 rps results in 0.1 V output
    public static final double PeakVoltage = 10.0;
    
    public static final double maxElevatorHeight = 1.53;

    public static final int kSensorChannel = 4;
    public static final boolean kSensorFalseIsEmpty = false;

    public static final double kElevatorIntakePosition = 0.0;
    public static final double kElevatorL2Position = 0.46; // 0.45-0.47??
    public static final double kElevatorL3Position = 0.93;
    public static final double kElevatorL4Position = 1.53;

    // The distance travelled for a single rotation of the Kraken output shaft.
    // 64 to 1 gear box, pulley is 47.75mm diameter (0.15m circumference),
    // two stage elevator moves twice the distance of the pulley.
    public static final double kElevatorGearRatio = (2 * 0.15)/64;

    // SmartDashboard update frequency for elevator subsystem state in 20ms counts.
    public static final int kTicksPerUpdate = 5;
  }
}
