// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeAlgaeCommand extends Command {

  private final AlgaeSubsystem m_subsystem;
  private int m_endCounter = -1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeAlgaeCommand(AlgaeSubsystem subsystem) {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setIntakeSpeed(AlgaeConstants.kIntakeSpeed);
    m_subsystem.setPosition(true);
    m_endCounter = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIntakeSpeed(0.0);
    m_subsystem.setPosition(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_subsystem.hasAlgae()) {
      m_endCounter++;
    }

    return (m_endCounter >= AlgaeConstants.kEndIntakeMaxCount);
  }
}
