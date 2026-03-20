// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticsConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_motorIntake;
  private DoubleSolenoid m_solenoid;
  private boolean m_motorRunning = false;
  private boolean m_Extended = false;
  private double m_MotorSpeed = IntakeConstants.kIntakeMotorSpeed;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(Boolean bHasPneumatics) throws Exception {
  
    // Create SmartDashboard items first so that these exist even
    // if we fail to create the subsystem.
    SmartDashboard.putBoolean("Intake Extended", m_Extended);
    SmartDashboard.putBoolean("Intake Running", m_motorRunning);

    // Test mode controls 
    SmartDashboard.putBoolean("Intake TestExtend", false);
    SmartDashboard.putBoolean("Intake TestRun", false);
    SmartDashboard.putNumber("Intake TestSpeed", m_MotorSpeed);
    
    m_motorIntake = new TalonFX(IntakeConstants.kIntakeMotorCANID);
    if(!m_motorIntake.isConnected())
    {
      throw new Exception("Intake motor is not connected!");
    }

    if(bHasPneumatics) {
      m_solenoid = new DoubleSolenoid(PneumaticsConstants.kPneumaticsHubCANID,
                                      PneumaticsConstants.kHubType, 
                                      IntakeConstants.kIntakeSolenoidForward,
                                      IntakeConstants.kIntakeSolenoidReverse);
    } else {
      throw new Exception("Intake subsystem requires pneumatics which are not present!");
    }
  }

  // Spins the shaft on the intake that will move fuel into the robot.
  public void start() {
    m_motorIntake.set(m_MotorSpeed);
    m_motorRunning = true;
    SmartDashboard.putBoolean("Intake Running", m_motorRunning);
  }

  // Stops spinning the intake shaft.
  public void stop() {
    m_motorIntake.set(0.0);
    m_motorRunning = false;
    SmartDashboard.putBoolean("Intake Running", m_motorRunning);
  }
  public void extend() {
    m_solenoid.set(IntakeConstants.kIntakeExtend);
    m_Extended = true;
    SmartDashboard.putBoolean("Intake Extended", m_Extended);
  }

  // Retracts the intake mechanism over the bumpers and inside of the robot.
  public void retract() {
    m_solenoid.set(IntakeConstants.kIntakeRetract);
    m_Extended = false;
    SmartDashboard.putBoolean("Intake Extended", m_Extended);
  }

    // Retracts the intake mechanism over the bumpers and inside of the robot.
  public void pneumatics_off() {
    m_solenoid.set(DoubleSolenoid.Value.kOff);
  }

  // Returns true if the intake shaft is spinning.
  public boolean isIntakeStarted() {
    return m_motorRunning;
  }

  // Returns true if the intake is extended outside of the frame perimiter.
  public boolean isIntakeExtended() {
    return m_Extended;
  }

  public void setIntakeSpeed(double speed) {
    m_MotorSpeed = speed;
    if(m_motorRunning)
    {
      start();
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void testPeriodic() {
    // Set the motor states based on the SmartDashboard test controls.
    Boolean Extend = SmartDashboard.getBoolean("Intake TestExtend", false);
    Boolean Run = SmartDashboard.getBoolean("Intake TestRun", false);
    double Speed = SmartDashboard.getNumber("Intake TestSpeed", m_MotorSpeed);

    setIntakeSpeed(Speed);
    
    // Turn the intake on or off depending upon test control, only changing the state if the control actually changed.
    if(Run) {
      if(!m_motorRunning) {
        this.start();
      }
    }
    else {
        if(m_motorRunning) {
        this.stop();
      }  
    }
    if(Extend) {
      if(!m_Extended) {
        this.extend();
      }
    }
    else {
        if(m_Extended) {
        this.retract();
      }  
    }
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
