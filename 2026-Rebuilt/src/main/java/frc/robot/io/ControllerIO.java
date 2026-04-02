// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public interface ControllerIO {
  @AutoLog
  public static class ControllerIOInputs {
    public double rightJoystickY = 0.0;
    public double rightJoystickX = 0.0;
    public double leftJoystickY = 0.0;
    public double leftJoystickX = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ControllerIOInputs inputs, CommandXboxController controller) {}
}
