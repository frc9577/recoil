// This Supports setting a Double Solenoid to a set position.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/** An example command that uses an example subsystem. */
public class ClimbCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem m_subsystem;
  private ClimbSubsystem.State m_position = ClimbSubsystem.State.OFF;

  /**
   * Creates a new ClimbCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbCommand(ClimbSubsystem subsystem, ClimbSubsystem.State position) {
    m_subsystem = subsystem;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setClimbState(m_position);
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
