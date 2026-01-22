// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class ArcadeDriveCommandNoPID extends Command {

  private final DriveSubsystem m_subsystem;
  private CommandXboxController m_driveController;

  /**
   * Creates a new ArcadeDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDriveCommandNoPID(
    DriveSubsystem subsystem,
    CommandXboxController driveController
  ) {
    m_subsystem = subsystem;
    m_driveController = driveController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Note: We negate both axis values so that pushing the joystick forwards
    // (which makes the readin more negative) increases the speed and twisting clockwise
    // turns the robot clockwise.
    double speedInput = -m_driveController.getRightY();
    double turnInput = m_driveController.getLeftX();

    // Set the deadband
    if (Math.abs(speedInput) < OperatorConstants.kDriverControllerDeadband) {
      speedInput = 0;
    }
    if (Math.abs(turnInput) < OperatorConstants.kDriverControllerDeadband) {
      turnInput = 0;
    }

    double leftSpeed = speedInput + turnInput;
    double rightSpeed = speedInput - turnInput;

    // Find the maximum possible value of (throttle + turn) along the vector
    // that the joystick is pointing, then desaturate the wheel speeds
    double greaterInput = Math.max(Math.abs(speedInput), Math.abs(turnInput));
    double lesserInput = Math.min(Math.abs(speedInput), Math.abs(turnInput));
    if (greaterInput == 0.0) {
      m_subsystem.setDifferentialSpeedNoPid(0, 0);
    } else {
      double saturatedInput = (greaterInput + lesserInput) / greaterInput;
      leftSpeed /= saturatedInput;
      rightSpeed /= saturatedInput;

      m_subsystem.setDifferentialSpeedNoPid(leftSpeed, rightSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
