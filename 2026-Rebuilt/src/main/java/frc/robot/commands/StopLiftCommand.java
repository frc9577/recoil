package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * 
 * Stop the launcher lift motor and spin down the flywheel.
 * 
 **/
public class StopLiftCommand extends Command {
  private final LauncherSubsystem m_subsystem;
  
  /**
   * Creates a new StopLiftCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StopLiftCommand(LauncherSubsystem subsystem) 
  {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.stopLift();
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
        return true;
  }
}
