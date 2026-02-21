package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * 
 * Stop the launcher lift motor and spin down the flywheel.
 * 
 **/
public class StartFlywheelCommand extends Command {
  private final LauncherSubsystem m_subsystem;
  private final double m_targetSpeedrpm;
  private final double m_Tolerancerpm;
  
  /**
   * Creates a new StartFlywheelCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StartFlywheelCommand(LauncherSubsystem subsystem, double TargetSpeedrpm, double Tolerancerpm) 
  {
    m_subsystem = subsystem;
    m_targetSpeedrpm = TargetSpeedrpm;
    m_Tolerancerpm = Tolerancerpm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setTargetSpeedrpm(m_targetSpeedrpm);
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
    // The command signals end when the flywheel speed is within the tolerance of the target.
    double currentSpeed = m_subsystem.getCurrentSpeedrpm();
    if (Math.abs(currentSpeed - m_targetSpeedrpm) <= m_Tolerancerpm)
    {
        return true;
    }
    else
    {
        return false;
    }
  }
}
