package frc.robot.commands.autoCommands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.util.AutoFromList;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class BackupAndShoot extends Command {
  // Pass-ins
  private final DriveSubsystem m_DriveSubsystem;
  private final DifferentialDrivePoseEstimator m_PoseEstimator;
  private final BooleanSupplier m_isRed;
  private final PathConstraints m_constraints;

  /**
   * Creates a new BackupAndShoot.
   */
  public BackupAndShoot(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed, PathConstraints constraints) {
    m_DriveSubsystem = driveSubsystem;
    m_PoseEstimator = poseEstimator;
    m_isRed = isRed;
    m_constraints = constraints;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Init the new sequence
    ArrayList<Object> sequence = new ArrayList<Object>(Arrays.asList(
      //new InstantCommand(), // Spin-up flywheel
      new DriveInALineFromPos(m_PoseEstimator, 2, true),

      // Shoot all balls \\
      new AimAtHub(m_DriveSubsystem, m_PoseEstimator, 2.0, m_isRed)//, // Aim at hub
      //new InstantCommand(), // Full engage flywheel
      //new StartLiftCommand(null),

      // Some sort of wait command \\

      // Ramp down flywheel \\
      //new StopLiftCommand(null),
      //new InstantCommand() // Disable flywheel
    ));

    // Run the command
    Command auto = new AutoFromList(sequence, m_constraints, m_DriveSubsystem, m_PoseEstimator, false, false);
    CommandScheduler.getInstance().schedule(auto);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
