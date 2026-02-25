// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;

/** An example command that uses an example subsystem. */
public class DifferentialDriveCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private CommandXboxController m_driveController;

  /**
   * Creates a new ArcadeDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DifferentialDriveCommand(DriveSubsystem subsystem, CommandXboxController driveController)  {
    m_subsystem = subsystem;
    m_driveController = driveController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Note: We negate both axis values so that pushing the joystick forwards
    // (which makes the readin more negative) increases the speed and twisting clockwise
    // turns the robot clockwise.
    double leftSpeed  = -m_driveController.getLeftY();
    double rightSpeed = -m_driveController.getRightY();

    // Set the deadband
    if (Math.abs(leftSpeed) < OperatorConstants.kDriverControllerDeadband) {
      leftSpeed = 0;
    }
    if (Math.abs(rightSpeed) < OperatorConstants.kDriverControllerDeadband) {
      rightSpeed = 0;
    }

    leftSpeed  *= DrivetrainConstants.kMaxVelocityMPS;
    rightSpeed *= DrivetrainConstants.kMaxVelocityMPS;

    m_subsystem.setDifferentialSpeeds( leftSpeed, rightSpeed );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
