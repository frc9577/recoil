// This Supports setting a motor to a set speed.

package frc.robot.commands;


import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class MotorConstSpeedCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Subsystem m_subsystem;
  private MotorController m_motor;
  private double m_speed = 0.0;

  /**
   * Creates a new WinchStartCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MotorConstSpeedCommand(Subsystem subsystem, MotorController motor, double speed) {
    m_subsystem = subsystem;
    m_motor = motor;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_motor.set(m_speed);
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
