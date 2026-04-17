package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.Pigeon;

public class SetPidgeonToLimelight extends Command {
  private final Pigeon m_pigeon;
  private final LimelightSubsystem m_limelight;
  
  /**
   * Creates a new SetPidgeonToLimelight.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetPidgeonToLimelight(Pigeon pigeon, LimelightSubsystem limelight) 
  {
    m_pigeon = pigeon;
    m_limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Double pigeonYaw = m_pigeon.getYaw();
    Double limelightYaw = m_limelight.getRobotYaw();

    if (limelightYaw != null && pigeonYaw != null) {
      Double error = limelightYaw - pigeonYaw;
      Double newYaw = pigeonYaw + (0.5 * error);
      m_pigeon.setYaw(newYaw);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The command signals end immediately.
    return true;
  }
}
