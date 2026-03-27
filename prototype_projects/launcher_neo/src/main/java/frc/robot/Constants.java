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

  public static class LauncherConstants {
    public static final int kMotorCANID = 50;

    // These numbers came from the ctre example then tweaked
    public static final double kP = 0.0001; // An error of 1 rotation results in x V output
    public static final double kI = 0.0;
    public static final double kD = 0.0; // A velocity of 1 rps results in x V output
    public static final double kV = (12.0/40000.0); // Expected value is (12.0/5767) but this yields a huge offset (100rpm setpoint, 700rpm actual)
    public static final double kCruiseVelocityRPM = 5700.0; // Note actually needed during velocity control.
    public static final double kAllowedErrorR = 10.0;
    public static final double kMaxAccelerationRPMPS = 1000.0;
  }
}
