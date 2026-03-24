package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerBulkSubsystem;
import frc.robot.subsystems.LiftSubsystem;

/**
 * 
 * Start shooting fuel by starting the lift motor and both motors in the indexer/bulk subsystem.
 * 
 **/
public class StartShootCommand extends Command {
  private final LiftSubsystem m_Lift;
  private final IndexerBulkSubsystem m_IndexerBulk;
  
  /**
   * Creates a new StartShootCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StartShootCommand(LiftSubsystem lift, IndexerBulkSubsystem indexerbulk) 
  {
    m_Lift = lift;
    m_IndexerBulk = indexerbulk;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Lift);
    addRequirements(m_IndexerBulk);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IndexerBulk.startBulkTransfer();
    m_IndexerBulk.startIndexer();
    m_Lift.startLift();
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
