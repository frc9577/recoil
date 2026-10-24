package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerBulkSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.utils.HubUtils;
import frc.robot.utils.LauncherUtils;

/**
 * 
 * Start the launcher flywheel and get it up to the target speed.
 * 
 **/
public class LauncherSequence extends Command {
  private final LauncherSubsystem m_launcherSubsystem;
  private final LiftSubsystem m_liftSubsystem;
  private final IndexerBulkSubsystem m_indexerBulkSubsystem;

  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final BooleanSupplier m_isRed;
  private final double m_toleranceRPM;

  private double m_targetSpeedrpm = 0.0;
  private boolean m_launcherAtSpeed = false;
  
  /**
   * Creates a new LauncherSequence.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LauncherSequence(LauncherSubsystem launcherSubsystem, LiftSubsystem liftSubsystem, IndexerBulkSubsystem indexerBulkSubsystem, DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed, double toleranceRPM) 
  {
    m_launcherSubsystem = launcherSubsystem;
    m_liftSubsystem = liftSubsystem;
    m_indexerBulkSubsystem = indexerBulkSubsystem;

    m_poseEstimator = poseEstimator;
    m_isRed = isRed;
    m_toleranceRPM = toleranceRPM;
    m_targetSpeedrpm = 0.0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcherSubsystem);
    addRequirements(m_liftSubsystem);
    addRequirements(m_indexerBulkSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set the speed based off of the distance from the hub.
    double Distance = HubUtils.getHubDistance(m_poseEstimator, m_isRed);
    m_targetSpeedrpm = LauncherUtils.getFlywheelSpeed(Distance);
    m_launcherSubsystem.setTargetSpeedrpm(m_targetSpeedrpm);

    // Check if its up to speed.
    double targetSpeed = m_launcherSubsystem.getTargetSpeedrpm();
    double currentSpeed = m_launcherSubsystem.getCurrentSpeedrpm();
    m_launcherAtSpeed = Math.abs(currentSpeed - targetSpeed) <= m_toleranceRPM;

    // Run the feed based off of if the launcher is at speed.
    if (m_launcherAtSpeed) {
      m_liftSubsystem.startLift();
      m_indexerBulkSubsystem.startIndexer();
      m_indexerBulkSubsystem.startBulkTransfer();
    } else {
      m_indexerBulkSubsystem.stopBulkTransfer();
      m_indexerBulkSubsystem.stopIndexer();
      m_liftSubsystem.stopLift();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerBulkSubsystem.stopBulkTransfer();
    m_indexerBulkSubsystem.stopIndexer();
    m_liftSubsystem.stopLift();
    m_launcherSubsystem.setTargetSpeedrpm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
