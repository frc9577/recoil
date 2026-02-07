// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // TODO: write this
  }

  // Spins the shaft on the intake that will move fuel into the robot.
  public void start() {
    // TODO: implement start
  }

  // Stops spinning the intake shaft.
  public void stop() {
    // TODO: implement stop

  // Extends the intake mechanism over the bumpers and outside of the robot.
  }
  public void extend() {
    // TODO: implement extend
  }

  // Retracts the intake mechanism over the bumpers and inside of the robot.
  public void retract() {
    // TODO: implement retract
  }

  // Returns true if the intake shaft is spinning.
  public boolean isIntakeStarted() {
    // TODO: implement isIntakeStarted
    return false;
  }

  // Returns true if the intake is extended outside of the frame perimiter.
  public boolean isIntakeExtended() {
    // TODO: implement isIntakeExtended
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
