package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerBulkSubsystem;

/**
 * 
 * Start shooting fuel by starting the lift motor and both motors in the indexer/bulk subsystem.
 * 
 **/
public class UnjamJiggleCommand extends Command {
  private final DriveSubsystem m_DriveSubsystem;
  private final IndexerBulkSubsystem m_IndexerBulk;
  
  private int m_switchTick = 50; // Switches every second
  private int m_executeTick = 0;
  private boolean m_isForward = false; // switches instantly, so forward first.

  /**
   * Creates a new ReverseIndexBulk.
   *
   * @param subsystem The subsystem used by this command.
   */
  public UnjamJiggleCommand(DriveSubsystem driveSubsystem, IndexerBulkSubsystem indexerbulk) 
  {
    m_DriveSubsystem = driveSubsystem;
    m_IndexerBulk = indexerbulk;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
    addRequirements(m_IndexerBulk);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_executeTick = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_executeTick % m_switchTick == 0) {
      m_isForward = !m_isForward;
    }

    double targetSpeed;
    if(m_isForward) {
      targetSpeed = 2.0;
    } else {
      targetSpeed = -2.0;
    }

    m_DriveSubsystem.setDifferentialSpeeds(targetSpeed, targetSpeed);
    m_IndexerBulk.reverseBulkTransfer();
    m_IndexerBulk.reverseIndexer();

    m_executeTick++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.setDifferentialSpeeds(0.0, 0.0);
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
