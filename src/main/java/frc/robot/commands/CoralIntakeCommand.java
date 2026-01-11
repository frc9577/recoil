// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CoralIntakeCommand extends Command {
  private final CoralSubsystem m_subsystem;
  private boolean m_hasTriggered = false;
  private int m_endCounter = -1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CoralIntakeCommand(CoralSubsystem subsystem) {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setIntakeSpeed(CoralConstants.kMotorIntakeSpeed);
    m_subsystem.setOutputSpeed(CoralConstants.kMotorOutputSpeed);

    m_endCounter = -1;
    m_hasTriggered = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIntakeSpeed(0.0);
    m_subsystem.setOutputSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_subsystem.hasCoral() == true && m_hasTriggered == false) {
      m_hasTriggered = true;
    }
    
    if (m_hasTriggered == true) {
        m_endCounter++;
    }

    return (m_endCounter >= CoralConstants.kEndIntakeMaxCount);
  }
}
