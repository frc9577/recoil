// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.io;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.DriveSubsystem;

public class DriveIOTalonFX implements DriveIO {
  public DriveIOTalonFX() {

  }

  @Override
  public void updateCurrentValues(DriveIOInputs inputs, TalonFX leftLead, TalonFX rightLead) {
    inputs.leftLeadExists = true;
    inputs.leftLeadCurrentSpeed = DriveSubsystem.ConvertRotationsToMeters(leftLead.getVelocity().getValueAsDouble());
    inputs.leftLeadDistance = DriveSubsystem.ConvertRotationsToMeters(leftLead.getPosition().getValueAsDouble());
    inputs.leftLeadTempature = leftLead.getDeviceTemp().getValueAsDouble();

    inputs.rightLeadExists = true;
    inputs.rightLeadCurrentSpeed = DriveSubsystem.ConvertRotationsToMeters(rightLead.getVelocity().getValueAsDouble());
    inputs.rightLeadDistance = DriveSubsystem.ConvertRotationsToMeters(rightLead.getPosition().getValueAsDouble());
    inputs.rightLeadTempature = rightLead.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void updateCurrentValues(DriveIOInputs inputs, TalonFX leftLead, TalonFX leftFollower, TalonFX rightLead, TalonFX rightFollower) {
    updateCurrentValues(inputs, leftLead, rightLead);

    inputs.leftFollowerExists = true;
    inputs.leftFollowerCurrentSpeed = DriveSubsystem.ConvertRotationsToMeters(leftFollower.getVelocity().getValueAsDouble());
    inputs.leftFollowerDistance = DriveSubsystem.ConvertRotationsToMeters(leftFollower.getPosition().getValueAsDouble());
    inputs.leftFollowerTempature = leftFollower.getDeviceTemp().getValueAsDouble();

    inputs.rightFollowerExists = true;
    inputs.rightFollowerCurrentSpeed = DriveSubsystem.ConvertRotationsToMeters(rightFollower.getVelocity().getValueAsDouble());
    inputs.rightFollowerDistance = DriveSubsystem.ConvertRotationsToMeters(rightFollower.getPosition().getValueAsDouble());
    inputs.rightFollowerTempature = rightFollower.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void updateSetValues(DriveIOInputs inputs, double leftSetRPS, double rightSetRPS) {
    inputs.leftLeadTargetSpeed = leftSetRPS;
    if (inputs.leftFollowerExists == true) {
      inputs.leftFollowerTargetSpeed = leftSetRPS;
    }

    inputs.rightLeadTargetSpeed = rightSetRPS;
    if (inputs.rightFollowerExists == true) {
      inputs.rightFollowerTargetSpeed = rightSetRPS;
    }
  }
}
