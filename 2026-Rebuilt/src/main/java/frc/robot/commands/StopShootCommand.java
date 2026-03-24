package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerBulkSubsystem;
import frc.robot.subsystems.LiftSubsystem;

/**
 * 
 * Start shooting fuel by starting the launcher lift motor and both motors in the indexer/bulk subsystem.
 * 
 **/
public class StopShootCommand extends Command {
  private final LiftSubsystem m_Lift;
  private final IndexerBulkSubsystem m_IndexerBulk;
  
  /**
   * Stops shooting by stopping the lift motor and both motors in bulk/indexer.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StopShootCommand(LiftSubsystem lift, IndexerBulkSubsystem indexerbulk) 
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
    m_IndexerBulk.stopBulkTransfer();
    m_IndexerBulk.stopIndexer();
    m_Lift.stopLift();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // This command ends immediately.
  @Override
  public boolean isFinished() {
        return true;
  }
}
