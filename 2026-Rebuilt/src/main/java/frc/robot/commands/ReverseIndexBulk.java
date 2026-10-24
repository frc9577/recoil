package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerBulkSubsystem;

/**
 * 
 * Start shooting fuel by starting the lift motor and both motors in the indexer/bulk subsystem.
 * 
 **/
public class ReverseIndexBulk extends Command {
  private final IndexerBulkSubsystem m_IndexerBulk;
  
  /**
   * Creates a new ReverseIndexBulk.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReverseIndexBulk(IndexerBulkSubsystem indexerbulk) 
  {
    m_IndexerBulk = indexerbulk;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IndexerBulk);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IndexerBulk.reverseBulkTransfer();
    m_IndexerBulk.reverseIndexer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // This command only ends when another command is scheduled on the
  // subsystem.
  @Override
  public boolean isFinished() {
        return false;
  }
}
