// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticsConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_motorIntake;
  private Solenoid m_solenoid;
  private boolean m_motorRunning = false;
  private boolean m_Extended = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() throws Exception {
    m_motorIntake = new TalonFX(IntakeConstants.kIntakeMotorCANID);
    if(!m_motorIntake.isConnected())
    {
      throw new Exception("Intake motor is not connected!");
    }

    m_solenoid = new Solenoid(PneumaticsConstants.kPneumaticsHubCANID,
                              PneumaticsConstants.kHubType, 
                              IntakeConstants.kIntakeSolenoid);

    SmartDashboard.putBoolean("Intake Extended", m_Extended);
    SmartDashboard.putBoolean("Intake Running", m_motorRunning);
  }

  // Spins the shaft on the intake that will move fuel into the robot.
  public void start() {
    m_motorIntake.set(IntakeConstants.kIntakeMotorSpeed);
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

  // Returns true if the intake shaft is spinning.
  public boolean isIntakeStarted() {
    return m_motorRunning;
  }

  // Returns true if the intake is extended outside of the frame perimiter.
  public boolean isIntakeExtended() {
    return m_Extended;
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
