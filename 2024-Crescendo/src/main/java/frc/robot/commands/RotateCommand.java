// This Supports setting a Double Solenoid to a set position.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GooseRotationSubsystem;

/** An example command that uses an example subsystem. */
public class RotateCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final GooseRotationSubsystem m_subsystem;
  private double m_angle = 0.0;

  /**
   * Creates a new RotateCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateCommand(GooseRotationSubsystem subsystem, double angle) {
    m_subsystem = subsystem;
    m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setSetPointAngle(m_angle);
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
