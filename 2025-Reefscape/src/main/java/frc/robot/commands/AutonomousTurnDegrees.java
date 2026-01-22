// This command is left intentionally empty.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class AutonomousTurnDegrees extends Command {

  // clockwise is positive
  DriveSubsystem m_Subsystem;
  float m_Degrees;
  double m_Speed;

  float m_FinalDegrees;
  boolean m_MovingClockwise;

  /**
   * Creates a new AutonomousTurnDegrees
   */
  public AutonomousTurnDegrees(
    DriveSubsystem subsystem,
    float degrees,
    double speed
  ) {
    m_Subsystem = subsystem;
    m_Degrees = degrees;
    m_Speed = Math.abs(speed);

    m_FinalDegrees = subsystem.getGyroYaw() + degrees;
    m_MovingClockwise = degrees > 0;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_MovingClockwise) {
      m_Subsystem.setTankSpeeds(m_Speed, -m_Speed);
    } else {
      m_Subsystem.setTankSpeeds(-m_Speed, m_Speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_MovingClockwise) {
      m_Subsystem.setTankSpeeds(m_Speed, -m_Speed);
    } else {
      m_Subsystem.setTankSpeeds(-m_Speed, m_Speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Subsystem.setTankSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    float turnError = m_FinalDegrees - m_Subsystem.getGyroYaw();

    System.out.println("Turning Clockwise? " + m_MovingClockwise);
    System.out.println("Final Degrees: " + m_FinalDegrees);
    System.out.println("Error: " + turnError);

    if (
      (m_MovingClockwise && (turnError <= 0)) ||
      (!m_MovingClockwise && (turnError >= 0))
    ) {
      return true;
    } else {
      return false;
    }
  }
}
