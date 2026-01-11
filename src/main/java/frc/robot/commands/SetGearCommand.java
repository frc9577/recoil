// This supports setting the robot drivetrain to low or high gear.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SetGearCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private boolean m_lowGear = false;

  /**
   * Creates a new SetGearCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetGearCommand(DriveSubsystem subsystem, boolean lowGear) {
    m_subsystem = subsystem;
    m_lowGear = lowGear;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setLowGear(m_lowGear);
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
