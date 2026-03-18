package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerBulkSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * 
 * Start shooting fuel by starting the launcher lift motor and both motors in the indexer/bulk subsystem.
 * 
 **/
public class StartShootCommand extends Command {
  private final LauncherSubsystem m_Launcher;
  private final IndexerBulkSubsystem m_IndexerBulk;
  
  /**
   * Creates a new StartShootCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StartShootCommand(LauncherSubsystem launcher, IndexerBulkSubsystem indexerbulk) 
  {
    m_Launcher = launcher;
    m_IndexerBulk = indexerbulk;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Launcher);
    addRequirements(m_IndexerBulk);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IndexerBulk.startBulkTransfer();
    m_IndexerBulk.startIndexer();
    m_Launcher.startLift();
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
