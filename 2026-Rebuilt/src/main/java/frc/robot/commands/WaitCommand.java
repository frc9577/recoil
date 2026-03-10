package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
/** An example command that uses an example subsystem. */
public class WaitCommand extends Command {
  private final double m_totalCalls;
  private double m_callCount;

  /**
   * Creates a new WaitCommand.
   */
  public WaitCommand(double seconds) 
  {
    m_totalCalls = seconds * 50; // 50 calls per second.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting timer for " + String.valueOf(m_totalCalls));
    m_callCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_callCount++;
    System.out.println("Timer is now at call count " + String.valueOf(m_callCount));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Timer is now finished!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean check = m_callCount >= m_totalCalls;
    System.out.println(String.valueOf(m_callCount) + " >= " + String.valueOf(m_totalCalls) + ": " + String.valueOf(check));
    
    return check;
  }
}
