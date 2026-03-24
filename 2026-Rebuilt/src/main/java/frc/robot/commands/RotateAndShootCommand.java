package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.IndexerBulkSubsystem;

/** An example command that uses an example subsystem. */
public class RotateAndShootCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final LauncherSubsystem m_launcherSubsystem;
  private final LiftSubsystem m_liftSubsystem;
  private final IndexerBulkSubsystem m_indexerSubsystem;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final BooleanSupplier m_isRed;
  private final double m_flywheelTolerance;

  /**
   * Creates a new BackupAndShoot.
   *
   * @param poseEstimator The pose estimator to get the location of the robot.
   * @param targetPose The pose the robot will make a point to go to.
   * @param constraints The constraints the robot will follow while following the path. 
   */
  public RotateAndShootCommand(DriveSubsystem driveSubsystem, LauncherSubsystem launcherSubsystem, LiftSubsystem liftSubsystem, IndexerBulkSubsystem indexerSubsystem, DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed, double flywheelTolerance) 
  {
    m_driveSubsystem = driveSubsystem;
    m_launcherSubsystem = launcherSubsystem;
    m_liftSubsystem = liftSubsystem;
    m_indexerSubsystem = indexerSubsystem;
    m_poseEstimator = poseEstimator;
    m_flywheelTolerance = flywheelTolerance;
    m_isRed = isRed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This should do the process to make the launcher get up to speed
    Command setLauncherSpeedCommand = new RangeLauncherCommand(m_launcherSubsystem, m_poseEstimator, m_flywheelTolerance, m_isRed);
    Command aimAtHubCommand = new AimAtHub(m_driveSubsystem, m_poseEstimator, 4.0, m_isRed);
    Command startShootCommand = new StartShootCommand(m_liftSubsystem, m_indexerSubsystem);

    ParallelCommandGroup TurnAndSet = new ParallelCommandGroup(setLauncherSpeedCommand, aimAtHubCommand);

    CommandScheduler.getInstance().schedule(
      TurnAndSet
      .andThen(startShootCommand)
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The command only ends when one of the subsystems it uses has another
    // command scheduled on it.
    return false;
  }
}
