package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.utils.HubUtils;
import frc.robot.utils.LauncherUtils;


/**
 * 
 * Stop the launcher motor and spin down the flywheel.
 * 
 **/
public class RangeLauncherCommand extends Command {
  private final BooleanSupplier m_isRed;
  private final LauncherSubsystem m_subsystem;
  private double m_tolerancerpm;
  private double m_targetSpeedrpm;
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  /**
   * Creates a new RangeLauncherCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RangeLauncherCommand(LauncherSubsystem subsystem, DifferentialDrivePoseEstimator poseEstimator, double Tolerancerpm, BooleanSupplier isRed) 
  {
    m_subsystem = subsystem;
    m_tolerancerpm = Tolerancerpm;
    m_poseEstimator = poseEstimator;
    m_isRed = isRed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double distance = HubUtils.getHubDistance(m_poseEstimator, m_isRed);
    m_targetSpeedrpm = LauncherUtils.getFlywheelSpeed(distance);

    m_subsystem.setTargetSpeedrpm(m_targetSpeedrpm);
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
    // The command signals end when the flywheel speed is within the tolerance of the target.
    double currentSpeed = m_subsystem.getCurrentSpeedrpm();
    if (Math.abs(currentSpeed - m_targetSpeedrpm) <= m_tolerancerpm)
    {
        return true;
    }
    else
    {
        return false;
    }
  }
}
