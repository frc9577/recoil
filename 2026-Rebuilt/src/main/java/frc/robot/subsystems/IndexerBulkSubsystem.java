//
// Subsystem offering low level control of the actuators and sensors
// in the indexer and bulk transport mechanisms.
//

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerBulkConstants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;


public class IndexerBulkSubsystem extends SubsystemBase {

  private TalonFXS m_motorBulk;
  private TalonFX m_motorIndexer;
  private Boolean m_bulkStarted = false;
  private Boolean m_indexerStarted = false;
  private double m_bulkSpeed = IndexerBulkConstants.kBulkMoveMotorSpeed;
  private double m_indexerSpeed = IndexerBulkConstants.kIndexerMotorSpeed;

  /** Creates a new IndexerBulkSubsystem. */
  public IndexerBulkSubsystem() throws Exception {

    // Create SmartDashboard items first so that these are populated
    // regardless of whether or not the subsystem is present.
    SmartDashboard.putBoolean("IndexerBulk BulkStarted", m_bulkStarted);
    SmartDashboard.putBoolean("IndexerBulk IndexerStarted", m_indexerStarted);
    SmartDashboard.putNumber("IndexerBulk BulkSpeed", m_bulkSpeed);
    SmartDashboard.putNumber("IndexerBulk IndexerSpeed", m_indexerSpeed);

    // Test mode controls
    SmartDashboard.putBoolean("IndexerBulk TestBulk", false);
    SmartDashboard.putBoolean("IndexerBulk TestIndexer", false);
    SmartDashboard.putNumber("IndexerBulk TestBulkSpeed", m_bulkSpeed);
    SmartDashboard.putNumber("IndexerBulk TestIndexerSpeed", m_indexerSpeed);

    m_motorBulk = new TalonFXS(IndexerBulkConstants.kBulkMoveMotorCANID);
    if (!m_motorBulk.isConnected())
    {
      throw new Exception("Bult transport motor is not present.");
    }

    m_motorIndexer = new TalonFX(IndexerBulkConstants.kIndexerMotorCANID);
    if (!m_motorIndexer.isConnected())
    {
      throw new Exception("Indexer motor is not present.");
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
  }

// Runs the belts that transfer the fuel from bulk storage to the indexer.
  public void startBulkTransfer() {
    m_motorBulk.set(m_bulkSpeed);
    m_bulkStarted = true;
    SmartDashboard.putBoolean("IndexerBulk BulkStarted", m_bulkStarted);
  }

// Stops the belts in bulk storage.
  public void stopBulkTransfer() {
    m_motorBulk.set(0.0);
    m_bulkStarted = false;
    SmartDashboard.putBoolean("IndexerBulk BulkStarted", m_bulkStarted);
  }

  public void setBulkTransferSpeed(double speed) {
    m_bulkSpeed = speed;
    if(m_bulkStarted)
    {
      startBulkTransfer();
    }
    SmartDashboard.putNumber("IndexerBulk BulkSpeed", m_bulkSpeed);
  }

  // Runs the indexer.
  public void startIndexer() {
    m_motorBulk.set(m_indexerSpeed);
    m_indexerStarted = true;
    SmartDashboard.putBoolean("IndexerBulk IndexerStarted", m_indexerStarted);
  }

// Stops the indexer.
  public void stopIndexer() {
    m_motorIndexer.set(0.0);
    m_indexerStarted = false;
    SmartDashboard.putBoolean("IndexerBulk IndexerStarted", m_indexerStarted);
  }

  public void setIndexerSpeed(double speed) {
    m_indexerSpeed = speed;
    if(m_indexerStarted)
    {
      startIndexer();
    }

    SmartDashboard.putNumber("IndexerBulk IndexerSpeed", m_indexerSpeed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void testPeriodic() {
    // Set the motor states based on the SmartDashboard test controls.
    Boolean BulkOn      = SmartDashboard.getBoolean("IndexerBulk TestBulk", false);
    Boolean IndexerOn   = SmartDashboard.getBoolean("IndexerBulk TestIndexer", false);
    double BulkSpeed    = SmartDashboard.getNumber("IndexerBulk TestBulkSpeed", m_bulkSpeed);
    double IndexerSpeed = SmartDashboard.getNumber("IndexerBulk TestIndexerSpeed", m_indexerSpeed);
    
    // Set new target speeds for each motor.
    setBulkTransferSpeed(BulkSpeed);
    setIndexerSpeed(IndexerSpeed);

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

    // Turn the indexer on or off depending upon test control, only changing the state if the control actually changed.
    if(IndexerOn) {
      if(!m_indexerStarted) {
        this.startIndexer();
      }
    }
    else {
        if(m_indexerStarted) {
        this.stopIndexer();
      }  
    }
  }
}
