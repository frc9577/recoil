//
// Subsystem offering low level control of the actuators and sensors
// in the indexer and bulk transport mechanisms.
//

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerBulkConstants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;


public class IndexerBulkSubsystem extends SubsystemBase {
  private final DigitalInput m_Sensor = new DigitalInput(IndexerBulkConstants.kLowerFuelSensorChannel);

  private TalonFXS m_motorBulk;
  private Boolean m_bulkStarted = false;

  /** Creates a new IndexerBulkSubsystem. */
  public IndexerBulkSubsystem() throws Exception {

    m_motorBulk = new TalonFXS(IndexerBulkConstants.kBulkMoveMotorCANID);
    if (!m_motorBulk.isConnected())
    {
      throw new Exception("Bult transport motor is not present.");
    }

    TalonFXSConfiguration motorConfig = new TalonFXSConfiguration();
    motorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motorBulk.getConfigurator().apply(motorConfig);
      System.out.println("bulk transfer config attempt #" + (i+1) + ": " + status.toString());
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs for bulk tranfer motor, error code: " + status.toString());
    }

    SmartDashboard.putBoolean("IndexerBulk BulkStarted", m_bulkStarted);
    SmartDashboard.putBoolean("IndexerBulk FuelPresent", false);

    // Test mode controls
    SmartDashboard.putBoolean("IndexerBulk TestBulk", false);
  }

// Runs the belts that transfer the fuel from bulk storage to the indexer.
  public void startBulkTransfer() {
    m_motorBulk.set(IndexerBulkConstants.kBulkMoveMotorSpeed);
    m_bulkStarted = true;
    SmartDashboard.putBoolean("IndexerBulk BulkStarted", m_bulkStarted);
  }

// Stops the belts in bulk storage.
  public void stopBulkTransfer() {
    m_motorBulk.set(0.0);
    m_bulkStarted = false;
    SmartDashboard.putBoolean("IndexerBulk BulkStarted", m_bulkStarted);
  }

// Returns true if lower fuel sensor detects fuel.
  public boolean isFuelPresent() {
      boolean sensorRead = m_Sensor.get();
      return (sensorRead == IndexerBulkConstants.kLowerFuelSensorIsEmpty) ? false : true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IndexerBulk FuelPresent", this.isFuelPresent());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void testPeriodic() {
    // Set the motor states based on the SmartDashboard test controls.
    Boolean BulkOn = SmartDashboard.getBoolean("IndexerBulk TestBulk", false);
    
    // Turn the bulk transfer on or off depending upon test control, only changing the state if the control actually changed.
    if(BulkOn) {
      if(!m_bulkStarted) {
        this.startBulkTransfer();
      }
    }
    else {
        if(m_bulkStarted) {
        this.stopBulkTransfer();
      }  
    }
  }
}
