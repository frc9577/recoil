package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerBulkSubsystem;

/**
 * 
 * Stop bulk move and indexer motors.
 * 
 **/
public class StopBulkMoveCommand extends Command {
  private final IndexerBulkSubsystem m_subsystem;
  
  /**
   * Creates a new StopBulkMoveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StopBulkMoveCommand(IndexerBulkSubsystem subsystem) 
  {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.stopBulkTransfer();
    m_subsystem.stopIndexer();
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
