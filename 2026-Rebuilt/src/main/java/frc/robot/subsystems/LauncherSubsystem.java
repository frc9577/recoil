// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  private double m_targetSpeedrpm;
  private boolean m_liftRunning;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // TODO: write this
  }

  //
  // Set the speed of the launcher flywheel in revolutions per minute.
  //
  public void setTargetSpeedrpm(double RPM)
  {
    m_targetSpeedrpm = RPM;

    // TODO: Populate this!
  }

  //
  // Get the current launcher flywheel speed in revolutions per minute.
  //
  public double getCurrentSpeedrpm()
  {
    // TODO: Populate this!
    return 0.0;
  }

  //
  // Get the current target speed for the launcher flywheel in revolutions
  // per minute.
  //
  public double getTargetSpeedrpm()
  {
    return m_targetSpeedrpm;
  }

  //
  // Start the lift mechanism motor.
  //
  public void startLift()
  {
    // TODO: Populate this!
  }

  //
  // Stop the lift mechanism motor.
  //
  public void stopLift()
  {
    // TODO: Populate this!
  }

  //
  // Determine whether or not a fuel is in position beneath the launcher
  // entrance.
  //
  public boolean isFuelAtLauncher()
  {
    // TODO: Populate this!
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
