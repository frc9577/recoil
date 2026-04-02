// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriverControllerIO implements ControllerIO {
  public DriverControllerIO() {

  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ControllerIOInputs inputs, CommandXboxController controller) {
    inputs.leftJoystickX = controller.getLeftX();
    inputs.leftJoystickY = controller.getLeftY();
    inputs.rightJoystickX = controller.getRightX();
    inputs.rightJoystickY = controller.getRightY();
  }
}
