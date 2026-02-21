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
import frc.robot.Constants.IndexerBulkConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;


public class IndexerBulkSubsystem extends SubsystemBase {
  private final DigitalInput m_Sensor = new DigitalInput(IndexerBulkConstants.kLowerFuelSensorChannel);

  private TalonFX m_motorIndexer;
  private TalonFXS m_motorBulk;

  /** Creates a new IndexerBulkSubsystem. */
  public IndexerBulkSubsystem() throws Exception {
    m_motorIndexer = new TalonFX(IndexerBulkConstants.kIndexerMotorCANID);
    if (!m_motorIndexer.isConnected())
    {
      throw new Exception("Indexer motor is not present.");
    }
    m_motorBulk = new TalonFXS(IndexerBulkConstants.kIndexerMotorCANID);
    if (!m_motorBulk.isConnected())
    {
      throw new Exception("Bult transport motor is not present.");
    }
  }

// Runs the belts that transfer the fuel from bulk storage to the indexer.
  public void startBulkTransfer() {
    m_motorBulk.set(IndexerBulkConstants.kBulkMoveMotorSpeed);
  }

// Stops the belts in bulk storage.
  public void stopBulkTransfer() {
    m_motorBulk.set(0.0);
  }

// Spins the indexer shaft which transfers fuel from the indexer to the launcher.
  public void startIndexer() {
    m_motorIndexer.set(IndexerBulkConstants.kIndexerMotorSpeed);
  }

// Stops spinning the indexer shaft.
  public void stopIndexer() {
    m_motorIndexer.set(0.0);
  }

// Returns true if lower fuel sensor detects fuel.
  public boolean isFuelPresent() {
      boolean sensorRead = m_Sensor.get();
      return (sensorRead == IndexerBulkConstants.kLowerFuelSensorIsEmpty) ? false : true;
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
