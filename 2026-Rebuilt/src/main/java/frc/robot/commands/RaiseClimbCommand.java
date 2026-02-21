package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbL1Subsystem;

/**
 * 
 * Raise the climb L1 mechanism hooks.
 * 
 **/
public class RaiseClimbCommand extends Command {
  private final ClimbL1Subsystem m_subsystem;
  
  /**
   * Creates a new RaiseClimbCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RaiseClimbCommand(ClimbL1Subsystem subsystem) 
  {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.raiseArms();
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
