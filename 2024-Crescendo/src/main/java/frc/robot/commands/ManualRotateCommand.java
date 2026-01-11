// This Supports setting a Double Solenoid to a set position.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.GooseRotationSubsystem;

/** An example command that uses an example subsystem. */
public class ManualRotateCommand extends Command {
  private final GooseRotationSubsystem m_subsystem;
  private XboxController m_joystick;

  /**
   * Creates a new ManualManualRotateCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualRotateCommand(GooseRotationSubsystem subsystem, XboxController joystick) {
    m_subsystem = subsystem;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setPointAngle = m_subsystem.getSetPointAngle();
    double joystickValue = -m_joystick.getLeftY();
    
    if ((joystickValue <= 0.25) && (joystickValue >= -0.25))
    { 
      return; 
    }

    if (joystickValue > 0.25)
    {
      setPointAngle += OperatorConstants.kGooseAngleIncrement;
    }
    else if (joystickValue < -0.25)
    {
      setPointAngle -= OperatorConstants.kGooseAngleIncrement;
    }

    if(setPointAngle > OperatorConstants.kGooseAngleMax)
    {
      setPointAngle = OperatorConstants.kGooseAngleMax;
    }
    else if (setPointAngle < OperatorConstants.kGooseAngleMin)
    {
      setPointAngle = OperatorConstants.kGooseAngleMin;
    }

    m_subsystem.setSetPointAngle(setPointAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
