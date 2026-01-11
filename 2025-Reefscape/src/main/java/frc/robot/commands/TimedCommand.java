// This should end up in common when that is set up - Owen G.

package frc.robot.commands;
import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class TimedCommand extends Command {
  private final Command m_command;
  private long m_startTime;
  private int m_executeTime;

  /**
   * Runs a command for a timed ammount.
   *
   * @param command The command being executed.
   * @param executeTime executeTime wants the run time in positive miliseconds.
   */
  public TimedCommand(Command command, int executeTime) 
  {
    m_command = command;

    if (executeTime < 0) {
      DriverStation.reportError("executeTime in TimedCommand Constructor is negative", null);
    } 
    m_executeTime = executeTime;

    // Gets the required subsystem dependencies and then adds them as a requirement.
    Set<Subsystem> requirements = m_command.getRequirements();
    for (Subsystem requirement : requirements) {
      addRequirements(requirement);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean timedOut = (System.currentTimeMillis() - m_startTime) >= m_executeTime;
    return timedOut || m_command.isFinished();
  }
}
