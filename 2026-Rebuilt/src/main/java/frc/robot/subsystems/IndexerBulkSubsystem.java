// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IndexerBulkSubsystem extends SubsystemBase {
  /** Creates a new IndexerBulkSubsystem. */
  public IndexerBulkSubsystem() {
    // TODO: write this
  }

// Runs the belts that transfer the fuel from bulk storage to the indexer.
  public void startBulkTransfer() {
    // TODO: implement startBulkTransfer
  }

// Stops the belts in bulk storage.
  public void stopBulkTransfer() {
    //TODO: implement stopBulkTransfer
  }

// Spins the indexer shaft which transfers fuel from the indexer to the launcher.
  public void startIndexer() {
    // TODO: implement indexing
  }

// Stops spinning the indexer shaft.
  public void stopIndexer() {
    // TODO: implement stopIndexer
  }

// Returns true if lower fuel sensor detects fuel.
  public boolean isFuelPresent() {
    // TODO: implement isBeamBroken
    return false;
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
