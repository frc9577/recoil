// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class NoDriveCommand extends Command {
  private final DriveSubsystem m_subsystem;

  // Doing this because it needs to have a controller for the drive default command.
  @SuppressWarnings("unused")
  private CommandXboxController m_driveController;

  /**
   * Creates a new NoDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public NoDriveCommand(DriveSubsystem subsystem, CommandXboxController driveController)  {
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
    m_subsystem.setDifferentialSpeeds(
      0, 
      0
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
