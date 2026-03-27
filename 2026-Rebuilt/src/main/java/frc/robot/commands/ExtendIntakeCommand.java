package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * 
 * Extend the intake mechanism.
 * 
 **/
public class ExtendIntakeCommand extends Command {
  private final IntakeSubsystem m_subsystem;
  
  /**
   * Creates a new ExtendIntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExtendIntakeCommand(IntakeSubsystem subsystem) 
  {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This simple command extends the intake mechanism.
    m_subsystem.extend();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The command signals end immediately.
    return true;
  }
}
