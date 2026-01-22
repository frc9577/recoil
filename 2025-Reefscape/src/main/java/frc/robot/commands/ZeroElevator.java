// This is the command that zeros the elevator.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class ZeroElevator extends Command {

  private final ElevatorSubsystem m_subsystem;

  /**
   * Creates a new ZeroElevator.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZeroElevator(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setMotorSpeed(ElevatorConstants.kMotorZeroingSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setMotorSpeed(ElevatorConstants.kMotorZeroingSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotorSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.isElevatorDown();
  }
}
