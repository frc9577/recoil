package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.IndexerBulkSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LiftSubsystem;

/**
 * 
 * Start shooting fuel by starting the lift motor and both motors in the indexer/bulk subsystem.
 * 
 **/
public class ReverseEverything extends Command {
  private final IndexerBulkSubsystem m_IndexerBulk;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final LauncherSubsystem m_LauncherSubsystem;
  private final LiftSubsystem m_LiftSubsystem;
  
  /**
   * Creates a new ReverseIndexBulk.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ReverseEverything(IndexerBulkSubsystem indexerbulk, IntakeSubsystem intake, LauncherSubsystem launcher, LiftSubsystem liftSubsystem) 
  {
    m_IndexerBulk = indexerbulk;
    m_IntakeSubsystem = intake;
    m_LauncherSubsystem = launcher;
    m_LiftSubsystem = liftSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IndexerBulk);
    addRequirements(m_IntakeSubsystem);
    addRequirements(m_LauncherSubsystem);
    addRequirements(m_LiftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.reverse();
    m_IndexerBulk.reverseBulkTransfer();
    m_IndexerBulk.reverseIndexer();
    m_LiftSubsystem.setLiftSpeed(-0.5);
    m_LiftSubsystem.startLift();
    m_LauncherSubsystem.setTargetSpeedrpm(-500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LauncherSubsystem.setTargetSpeedrpm(0);
    m_LiftSubsystem.setLiftSpeed(0);
    m_LiftSubsystem.stopLift();
    m_LiftSubsystem.setLiftSpeed(LauncherConstants.kLiftMotorSpeed);
    m_IndexerBulk.stopIndexer();
    m_IndexerBulk.stopBulkTransfer();
    m_IntakeSubsystem.stop();
  }

  // This command only ends when another command is scheduled on the
  // subsystem.
  @Override
  public boolean isFinished() {
    return false;
  }
}
