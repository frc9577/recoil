package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LauncherSubsystem;
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
public class ManualFlywheelCommand extends Command {
  private final LauncherSubsystem m_subsystem;
  private final CommandXboxController m_controller;

  private final DifferentialDrivePoseEstimator m_PoseEstimator;
  private final BooleanSupplier m_isRed;
  private double m_targetSpeedrpm = 0.0;

  /**
   * Creates a new ManualFlywheelCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualFlywheelCommand(LauncherSubsystem subsystem, CommandXboxController controller, DifferentialDrivePoseEstimator poseEst, BooleanSupplier isRed) 
  {
    m_subsystem = subsystem;
    m_controller = controller;
    m_PoseEstimator = poseEst;
    m_isRed = isRed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  private double getJoystickSpeed() {
      // Deadband
      double joystickInput = -m_controller.getLeftY();
      if (Math.abs(joystickInput) <= 0.3) {
        joystickInput = 0;
      }

      // Add the change & Clamp
      double targetSpeedrpm = m_targetSpeedrpm + joystickInput;
      targetSpeedrpm = Math.min(3400.0, Math.max(2400.0, targetSpeedrpm));

      return targetSpeedrpm;
  }

  private double getDistanceSpeed() {
    // Calculate the current distance to the hub.
    double Distance = HubUtils.getHubDistance(m_PoseEstimator, m_isRed);

    // Determine flywheel speed to target hub from this distance.
    double targetSpeedrpm = LauncherUtils.getFlywheelSpeed(Distance);
    return targetSpeedrpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetSpeedrpm = getDistanceSpeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the speed depending if controller Y is pressed.
    if (m_controller != null && m_controller.y() != null && m_controller.y().getAsBoolean() == true) {
        m_targetSpeedrpm = getJoystickSpeed();
    } else {
        m_targetSpeedrpm = getDistanceSpeed();
    }

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
