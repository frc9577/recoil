// This is the default command for drive subsystem and handles jotstick control of the speed.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class TankDriveCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private XboxController m_tankController;

  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDriveCommand(DriveSubsystem subsystem, XboxController tankController) 
  {
    m_subsystem = subsystem;
    m_tankController = tankController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_subsystem.setTankSpeeds(-m_tankController.getLeftY(), -m_tankController.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}