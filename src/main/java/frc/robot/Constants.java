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
public final class Constants {
  public static class RobotConstants {
    public static final double minPnuematicsPressure = 80.0;
    public static final double maxPnuematicsPressure = 120.0;

    public static final int periodicTicksPerSecond = 50;
    public static final int pnuematicReportingFreq = 1;
    public static final int imuReportingFreq       = 1;
    public static final int gooseReportingFreq     = 1;
  }

  public static class AutoConstants {
    public static final long kPassLineDuration_mS = 2000;
    public static final double kPassLineSpeed = 0.40;
  }

  public static class DrivetrainConstants {
    public static final int kLeftFrontMotorCANID  = 10;
    public static final int kLeftBackMotorCANID   = 11;
    public static final int kRightFrontMotorCANID = 20;
    public static final int kRightBackMotorCANID  = 21;
  }

  public static class DriverConstants {
    public static final int kLeftDriveJoystick = 0;
    public static final int kRightDriveJoystick = 1;

    public static final double kTurnDivider = 2.0;
    public static final double kLowGearDivider = 2.0;

    // Left Joystick
    public static final int kLowGear = 1; // Joystick Trigger
    public static final int kReverseMode = 9; // Joystick button 9
    public static final int kForwardMode = 7; // Joystick button 7
    
    // Right Joystick
    public static final int kDriveStraight = 1; // Joystick Trigger
  }

  public static class OperatorConstants {
    public static final int kOperatorController = 2;

    public static final int kClimbUp        = 7; // Back Button
    public static final int kClimbDown      = 8; // Start Button

    //public static final int kIntakeAuto     = 3; // Button X
    public static final int kOutput         = 6; // Right Bumper
    public static final int kIntakeManual   = 5; // Left Bumper
    public static final int kFullStop       = 2; // Button B

    public static final int kMoveArmTop     = 4; // Button Y
    public static final int kMoveArmMiddle  = 3; // Button X
    public static final int kMoveArmBottom  = 1; // Button A

    public static final int kSpitOut = 10;  // right joystick down

    public static final double kGooseAngleIncrement = 1.0;
    public static final double kGooseAngleMax = 130.0;
    public static final double kGooseAngleMin = -10.0;
  }

  public static class ClimbConstants {
    public static final int kPneumaticsHubCANid = 1;

    // Temp Values, Needs to be changed when solunoid is set up onto the robot
    public static final int kExtendChannelL = 8;
    public static final int kRetractChannelL = 7;
    
    public static final int kExtendChannelR = 15;
    public static final int kRetractChannelR = 0;
  }

  public static class NoteHandlingConstants {
    public static final int kIntakeMotorCANID = 30;
    public static final int kOutputMotorCANID = 31;

    public static final int kNoteSensorChannel = 0;
    public static final boolean kNoteSensorNormallyOpen = true;

    public static final double kRollerSpeed = -0.6;
    public static final double kOutputRollerMultiplier = 1.3;

    public static final int kIntakeCurrentLimit = 50;
    public static final int kOutputCurrentLimit = 30;
  }

  public static class GooseRotationConstants {
    public static final int kRotateMotorCANID = 40;

    // TODO: Need to tune
    // SparkMax PID coefficients for the rotation controller.
    // These are currently taken from the REV example at https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
    public static final double kP         = 2.0;
    public static final double kI         = 0.0;
    public static final double kD         = 0.0;
    public static final double kIZone     = 0.0;
    public static final double kFF        = 0.0;
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;

    // Starting in a non-end positon, there will be a switch at one or both of the end positions.
    // When the switch(s) are hit the software will reset to the known position.
    // Currently we are presuming there will be a thing to set the arm at the start of the game.
    // TODO: The next 7 values are guesses and need to be measured.
    public static final double kStartingAngle         = 30.0;

    public static final double kBottomRawEncoderValue = 0.680;
    public static final double kBottomAngle           = -10.0;

    public static final double kTopRawEncoderValue    = 0.386;
    public static final double kTopAngle              = 115.0; 

    public static final double kScoreAngle            = 130.0;
    public static final double kMiddleAngle           = 40.0;
    public static final double kIntakeAngle           = -5.7;

    // This defines the gear ratio between the motor and output shafts. Divide
    // the motor rotation count by this number to determine the output shaft
    // rotation count.
    public static final int kMotorCurrentLimit        = 30;
  }
}
