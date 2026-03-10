package frc.robot.commands.autoCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AimAtHub;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class BackupAndShoot extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final BooleanSupplier m_isRed;

  /**
   * Creates a new BackupAndShoot.
   *
   * @param poseEstimator The pose estimator to get the location of the robot.
   * @param targetPose The pose the robot will make a point to go to.
   * @param constraints The constraints the robot will follow while following the path. 
   */
  public BackupAndShoot(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed) 
  {
    m_driveSubsystem = driveSubsystem;
    m_poseEstimator = poseEstimator;
    m_isRed = isRed;
  }

  // Called when the command is initially scheduled.
  // TODO: Implement Launcher!
  @Override
  public void initialize() {
    // This should do the process to make the launcher get up to speed
    Command speedUpLauncher = new InstantCommand(() -> System.out.println("Speeding up launcher!"));

    Command deadreckonBackward = new DeadreckonDistance(m_driveSubsystem, 1.5, -2.0);
    Command aimCommand = new AimAtHub(m_driveSubsystem, m_poseEstimator, 4.0, m_isRed);

    // This command should only end once the launcher is up to speed.
    Command waitForLauncherReady = new InstantCommand(() -> System.out.println("Waiting for launcher ready!"));

    // This command should only end once it is determined for the launcher ot be empty.
    Command fireLauncher = new InstantCommand(() -> System.out.println("Launching game peices!"));

    CommandScheduler.getInstance().schedule(
      speedUpLauncher
      .andThen(deadreckonBackward)
      .andThen(aimCommand)
      .andThen(waitForLauncherReady)
      .andThen(fireLauncher)
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
    // The command automatically ends
    return true;
  }
}
