package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** 
 * Turns the robot left at a fixed speed known to cause a stall. 
 * This command has no way to stop internally and must be cancled via another way.
*/
public class TurnLeftTest extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final double m_stallSpeed = 0.23;

  /**
   * Creates a new TurnLeftTest.
   *
   * @param driveSubsystem The driveSubsystem for the robot.
   */
  public TurnLeftTest(DriveSubsystem driveSubsystem) 
  {
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting TurnLeftTest! This command must be cancled via an outside method!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.setDifferentialSpeedNoPid(-m_stallSpeed, m_stallSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setDifferentialSpeedNoPid(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
