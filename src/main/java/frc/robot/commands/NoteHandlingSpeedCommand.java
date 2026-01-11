// This Supports setting a motor to a set speed.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NoteHandlingConstants;
import frc.robot.subsystems.NoteHandlingSubsystem;

/** An example command that uses an example subsystem. */
public class NoteHandlingSpeedCommand extends Command {
  private final NoteHandlingSubsystem m_subsystem;
  private double m_speed = 0.0;
  private boolean m_isInput = true;

  /**
   * Creates a new WinchStartCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public NoteHandlingSpeedCommand(NoteHandlingSubsystem subsystem, double speed, boolean isInput) {
    m_subsystem = subsystem;
    m_speed = speed;
    m_isInput = isInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setIntakeSpeed(m_speed);
    if (!m_isInput)
    {
      m_subsystem.setOutputSpeed(-m_speed * NoteHandlingConstants.kOutputRollerMultiplier);
    }
    else
    {
       m_subsystem.setOutputSpeed(0.0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
