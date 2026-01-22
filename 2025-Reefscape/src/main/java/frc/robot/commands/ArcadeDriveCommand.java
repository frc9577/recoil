// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class ArcadeDriveCommand extends Command {

  private final DriveSubsystem m_subsystem;
  private XboxController m_driveController;

  /**
   * Creates a new ArcadeDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDriveCommand(
    DriveSubsystem subsystem,
    XboxController driveController
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
    m_subsystem.setArcadeSpeeds(
      -m_driveController.getRightY(),
      -m_driveController.getLeftX()
    );
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
