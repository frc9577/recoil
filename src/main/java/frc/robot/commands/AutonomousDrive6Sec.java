// This is the default auto command that drives the robot across the auto line.

package frc.robot.commands;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class AutonomousDrive6Sec extends Command {
  private final DriveSubsystem m_subsystem;
  private double m_leftSpeed = 0.0;
  private double m_rightSpeed = 0.0;
  private long m_startTime;

  /**
   * Creates a new AutonomousDrive6Sec.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousDrive6Sec(DriveSubsystem subsystem) 
  {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_subsystem.setTankSpeeds(0.0, 0.0);

    m_leftSpeed = AutoConstants.kPassLineSpeed;
    m_rightSpeed = AutoConstants.kPassLineSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setTankSpeeds(m_leftSpeed, m_rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setTankSpeeds(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= 6000;
  }
}
