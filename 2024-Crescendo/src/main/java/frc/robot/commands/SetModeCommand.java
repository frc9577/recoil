// This supports setting the robot drivetrain to normal or reverse mode.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SetModeCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private boolean m_reverseMode = false;

  /**
   * Creates a new SetModeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetModeCommand(DriveSubsystem subsystem, boolean reverseMode) {
    m_subsystem = subsystem;
    m_reverseMode = reverseMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setReverseMode(m_reverseMode);
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
    return true;
  }
}
