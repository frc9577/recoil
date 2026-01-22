package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

// TODO: Class header comment!

/** An example command that uses an example subsystem. */
public class CoralSpeedCommand extends Command {

  private final CoralSubsystem m_subsystem;
  private double m_intakeSpeed = 0.0;
  private double m_outputSpeed = 0.0;

  /**
   * Creates a new CoralSpeedCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CoralSpeedCommand(
    CoralSubsystem subsystem,
    double inputSpeed,
    double outputSpped
  ) {
    m_subsystem = subsystem;

    m_intakeSpeed = inputSpeed;
    m_outputSpeed = outputSpped;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setIntakeSpeed(m_intakeSpeed);
    m_subsystem.setOutputSpeed(m_outputSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIntakeSpeed(0);
    m_subsystem.setOutputSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
