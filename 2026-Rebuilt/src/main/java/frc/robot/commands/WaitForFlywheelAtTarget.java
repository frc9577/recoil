package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * 
 * Start the launcher flywheel and get it up to the target speed.
 * 
 **/
public class WaitForFlywheelAtTarget extends Command {
  private final LauncherSubsystem m_subsystem;
  private final double m_toleranceRPM;
  
  /**
   * Creates a new WaitForFlywheelAtTarget.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WaitForFlywheelAtTarget(LauncherSubsystem subsystem, double toleranceRPM) 
  {
    m_subsystem = subsystem;
    m_toleranceRPM = toleranceRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The command signals end when the flywheel speed is within the tolerance of the target.
    double targetSpeed = m_subsystem.getTargetSpeedrpm();
    double currentSpeed = m_subsystem.getCurrentSpeedrpm();
    if (Math.abs(currentSpeed - targetSpeed) <= m_toleranceRPM) {
      return true;
    } else {
      return false;
    }
  }
}
