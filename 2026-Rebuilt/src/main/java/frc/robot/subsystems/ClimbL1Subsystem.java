// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbL1Constants;
import frc.robot.Constants.PneumaticsConstants;

public class ClimbL1Subsystem extends SubsystemBase {
    private Solenoid m_LeftSolenoid;
    private Solenoid m_RightSolenoid;
    private boolean  m_bRaised = false;

  /** Creates a new ClimbL1Subsystem. */
  public ClimbL1Subsystem() {
    // Create member objects in the constructor in the hope that they throw exceptions
    // if underlying hardware is not present and, hence, allow our Optional system to
    // work.
    m_LeftSolenoid  = new Solenoid(PneumaticsConstants.kPneumaticsHubCANID,
                                   PneumaticsModuleType.REVPH, 
                                   ClimbL1Constants.kLeftSolenoidChannel);
    m_RightSolenoid = new Solenoid(PneumaticsConstants.kPneumaticsHubCANID,
                                   PneumaticsModuleType.REVPH, 
                                   ClimbL1Constants.kLeftSolenoidChannel);
    SmartDashboard.putBoolean("ClimbL1Raised", m_bRaised);
  }

  // Raise the climb arms.
  public void raiseArms()
  {
    m_LeftSolenoid.set(ClimbL1Constants.kClimbL1Raise);
    m_RightSolenoid.set(ClimbL1Constants.kClimbL1Raise);
    m_bRaised = true;
    SmartDashboard.putBoolean("ClimbL1Raised", m_bRaised);
  }

  // Lower the climb arms.
  public void lowerArms()
  {
    m_LeftSolenoid.set(ClimbL1Constants.kClimbL1Lower);
    m_RightSolenoid.set(ClimbL1Constants.kClimbL1Lower);
    m_bRaised = false;
    SmartDashboard.putBoolean("ClimbL1Raised", m_bRaised);
  }

  // Determine whether the arms are currently in the raised or lowered position.
  // NB: The robot has no way to know the current position of the arms so this merely
  // returns the last commanded state! Commands using this method may need to start a 
  // timer when this state changes to allow for the time taken to move the pneumatics.
  public boolean areArmsRaised()
  {
    return m_bRaised;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
