//
// Subsystem offering low level control of the launcher flywheel and
// fuel lift mechanisms.
//

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class LiftSubsystem extends SubsystemBase {
  private boolean m_liftRunning = false;

  private TalonFX m_motorLift;
  private double m_liftSpeed = LauncherConstants.kLiftMotorSpeed;

  /** Creates a new LiftSubsystem. 
     * @throws Exception */
    public LiftSubsystem() throws Exception {

    // Test mode controls
    SmartDashboard.putNumber("Lift TestLiftSpeed", LauncherConstants.kLiftMotorSpeed);
    SmartDashboard.putBoolean("Lift TestLiftRun", false);

      
    m_motorLift     = new TalonFX(LauncherConstants.kLauncherLiftMotorCANID);

    // Check that the launcher motors exist and throw an exception if they don't.
    if(!m_motorLift.isConnected())
    {
      throw new Exception("Lift motor is not present!");
    }

    m_motorLift.set(0.0);
  }

  //
  // Start the lift mechanism motor.
  //
  public void startLift()
  {
    m_motorLift.set(m_liftSpeed);
    m_liftRunning = true;
  }

  //
  // Stop the lift mechanism motor.
  //
  public void stopLift()
  {
    m_motorLift.set(0.0);
    m_liftRunning = false;
  }

  public void setLiftSpeed(double speed)
  {
    m_liftSpeed = speed;

    if(m_liftRunning)
    {
      startLift();
    }
  }

  // 
  // Determine whether or not the lift motor is running.
  public boolean isLiftMotorStarted()
  {
    return m_liftRunning;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void testPeriodic() {

    double liftspeed = SmartDashboard.getNumber("Lift TestLiftSpeed", 0.0 );
    Boolean liftRun = SmartDashboard.getBoolean("Lift TestLiftRun", false);
    
    this.setLiftSpeed(liftspeed);

    // Turn the intake on or off depending upon test control, only changing the state if the control actually changed.
    if(liftRun) {
      if(!m_liftRunning) {
        this.startLift();
      }
    }
    else {
        if(m_liftRunning) {
        this.stopLift();
      }  
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  
  }
}
