package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import frc.robot.utils.HubUtils;
import frc.robot.utils.LauncherUtils;

/**
 * 
 * This command, which runs until interrupted, constantly monitors the
 * distance between the robot and the hub and updates the launcher 
 * flywheel target speed to the correct value for the current position.
 * The speed is only modified if the distance is within the range where
 * it is physically possible for the robot to successfully launch fuel
 * into the hub. 
 * 
 **/
public class TrackHubFlywheelCommand extends Command {
  private final LauncherSubsystem m_subsystem;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final BooleanSupplier m_isRed;
  private double m_targetSpeedrpm = 0.0;

  /**
   * Creates a new TrackHubFlywheelCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TrackHubFlywheelCommand(LauncherSubsystem subsystem, DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed) 
  {
    m_subsystem = subsystem;
    m_poseEstimator = poseEstimator;
    m_isRed = isRed;
    m_targetSpeedrpm = 0.0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Calculate the current distance to the hub.
    double Distance = HubUtils.getHubDistance(m_poseEstimator, m_isRed);

    // Determine flywheel speed to target hub from this distance.
    m_targetSpeedrpm = LauncherUtils.getFlywheelSpeed(Distance);

    // Set new flywheel target speed.
    m_subsystem.setTargetSpeedrpm(m_targetSpeedrpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // This command never ends. It will be interrupted by the scheduler if
  // the operator chooses to run another command that requires the launcher
  // subsystem.
  @Override
  public boolean isFinished() {
        return false;
  }
}
