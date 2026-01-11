// This supports setting the robot drivetrain into straight driving mode while using tank mode.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SetDriveStraightCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private boolean m_straightMode = false;

  /**
   * Creates a new SetDriveStraightCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetDriveStraightCommand(DriveSubsystem subsystem, boolean straightMode) {
    m_subsystem = subsystem;
    m_straightMode = straightMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setStraightDriveMode(m_straightMode);
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
