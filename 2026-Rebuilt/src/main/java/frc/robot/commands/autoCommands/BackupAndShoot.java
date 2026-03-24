package frc.robot.commands.autoCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.commands.*;
import frc.robot.Constants.*;

/** An example command that uses an example subsystem. */
public class BackupAndShoot extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final LauncherSubsystem m_launcherSubsystem;
  private final LiftSubsystem m_liftSubsystem;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final BooleanSupplier m_isRed;

  /**
   * Creates a new BackupAndShoot.
   *
   * @param poseEstimator The pose estimator to get the location of the robot.
   * @param targetPose The pose the robot will make a point to go to.
   * @param constraints The constraints the robot will follow while following the path. 
   */
  public BackupAndShoot(DriveSubsystem driveSubsystem, LauncherSubsystem launcherSubsystem, LiftSubsystem liftSubsystem, DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed) 
  {
    m_driveSubsystem = driveSubsystem;
    m_launcherSubsystem = launcherSubsystem;
    m_liftSubsystem = liftSubsystem;
    m_poseEstimator = poseEstimator;
    m_isRed = isRed;
  }

  // Called when the command is initially scheduled.
  // TODO: Indexer Implemntation needed
  @Override
  public void initialize() {
    // This should do the process to make the launcher get up to speed
    Command rangeLauncher = new RangeLauncherCommand(m_launcherSubsystem, m_poseEstimator, LauncherConstants.kFlywheelToleranceRPM, m_isRed);
    Command keepSpeedCommand1 = new TrackHubFlywheelCommand(m_launcherSubsystem, m_poseEstimator, m_isRed);
    Command keepSpeedCommand2 = new TrackHubFlywheelCommand(m_launcherSubsystem, m_poseEstimator, m_isRed);

    Command deadreckonBackward = new DeadreckonDistance(m_driveSubsystem, 1.5, -2.0);
    Command aimCommand = new AimAtHub(m_driveSubsystem, m_poseEstimator, 4.0, m_isRed);

    Command startLift = new StartLiftCommand(m_liftSubsystem);

    SequentialCommandGroup fullComboCommand = new SequentialCommandGroup(
      new ParallelRaceGroup(
        new ParallelCommandGroup(
          rangeLauncher,
          keepSpeedCommand1
        ),
        new SequentialCommandGroup(
          deadreckonBackward,
          aimCommand,
          startLift // Replace with the combo launcher, indexer, mover
        )
      ),
      keepSpeedCommand2
    );

    CommandScheduler.getInstance().schedule(fullComboCommand);
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
    // The command automatically ends
    return true;
  }
}
