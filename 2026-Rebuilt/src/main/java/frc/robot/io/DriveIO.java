// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;

public interface DriveIO {
  @AutoLog
  public static class DriveIOInputs {
    public boolean leftLeadExists = false;
    public double leftLeadTargetSpeed = 0.0;
    public double leftLeadCurrentSpeed = 0.0;
    public double leftLeadDistance = 0.0;
    public double leftLeadTempature = 0.0;

    public boolean leftFollowerExists = false;
    public double leftFollowerTargetSpeed = 0.0;
    public double leftFollowerCurrentSpeed = 0.0;
    public double leftFollowerDistance = 0.0;
    public double leftFollowerTempature = 0.0;

    public boolean rightLeadExists = false;
    public double rightLeadTargetSpeed = 0.0;
    public double rightLeadCurrentSpeed = 0.0;
    public double rightLeadDistance = 0.0;
    public double rightLeadTempature = 0.0;

    public boolean rightFollowerExists = false;
    public double rightFollowerTargetSpeed = 0.0;
    public double rightFollowerCurrentSpeed = 0.0;
    public double rightFollowerDistance = 0.0;
    public double rightFollowerTempature = 0.0;
  }

  public default void updateCurrentValues(DriveIOInputs inputs, TalonFX leftLead, TalonFX rightLead) {}

  public default void updateCurrentValues(DriveIOInputs inputs, TalonFX leftLead, TalonFX leftFollower, TalonFX rightLead, TalonFX rightFollower) {}

  public default void updateSetValues(DriveIOInputs inputs, double leftSetRPS, double rightSetRPS) {}

}
