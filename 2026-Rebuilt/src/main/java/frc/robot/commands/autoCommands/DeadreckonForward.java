package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DeadreckonForward extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final double m_targetDistance;
  private final double m_speed;

  private double m_startDistance;

  /**
   * Creates a new DriveForward2mFromPos.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DeadreckonForward(DriveSubsystem driveSubsystem, double targetDistance, double speed) 
  {
    m_driveSubsystem = driveSubsystem;
    m_targetDistance = targetDistance;
    m_speed = speed;

    addRequirements(m_driveSubsystem);
  }

  private double avrgDistances() {
    double leftPos = m_driveSubsystem.getMotorPositionMeters(true);
    double rightPos = m_driveSubsystem.getMotorPositionMeters(false);
    return (leftPos + rightPos)/2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startDistance = avrgDistances();
    m_driveSubsystem.setDifferentialSpeeds(m_speed, m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.setDifferentialSpeeds(m_speed, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setDifferentialSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceDiff = avrgDistances() - m_startDistance;
    // System.out.println(String.valueOf(distanceDiff) + " --> " + String.valueOf(m_targetDistance) + " at " + String.valueOf(m_speed));

    if (distanceDiff >= m_targetDistance) {
      return true;
    } else {
      return false;
    }
  }
}
