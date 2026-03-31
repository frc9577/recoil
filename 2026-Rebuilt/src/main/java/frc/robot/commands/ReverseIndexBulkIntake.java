package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerBulkSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * 
 * Start shooting fuel by starting the lift motor and both motors in the indexer/bulk subsystem.
 * 
 **/
public class ReverseIndexBulkIntake extends Command {
  private final IndexerBulkSubsystem m_IndexerBulk;
  private final IntakeSubsystem m_IntakeSubsystem;
  
  /**
   * Creates a new ReverseIndexBulk.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReverseIndexBulkIntake(IndexerBulkSubsystem indexerbulk, IntakeSubsystem intake) 
  {
    m_IndexerBulk = indexerbulk;
    m_IntakeSubsystem = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IndexerBulk);
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IndexerBulk.reverseBulkTransfer();
    m_IndexerBulk.reverseIndexer();
    m_IntakeSubsystem.reverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop();
    m_IndexerBulk.stopBulkTransfer();
    m_IndexerBulk.stopIndexer();
  }

  // This command only ends when another command is scheduled on the
  // subsystem.
  @Override
  public boolean isFinished() {
    return false;
  }
}
