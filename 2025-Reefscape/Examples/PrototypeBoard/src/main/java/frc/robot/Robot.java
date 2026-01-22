// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * This is a sample program showing the use of the solenoid classes during operator control. Three
 * buttons from a joystick will be used to control two solenoids: One button to control the position
 * of a single solenoid and the other two buttons to control a double solenoid. Single solenoids can
 * either be on or off, such that the air diverted through them goes through either one channel or
 * the other. Double solenoids have three states: Off, Forward, and Reverse. Forward and Reverse
 * divert the air through the two channels and correspond to the on and off of a single solenoid,
 * but a double solenoid can also be "off", where the solenoid will remain in its default power off
 * state. Additionally, double solenoids take up two channels on your PCM whereas single solenoids
 * only take a single channel.
 */
public class Robot extends TimedRobot {

  private final Joystick m_stick = new Joystick(0);

  // DoubleSolenoid corresponds to a double solenoid.
  // In this case, it's connected to channels 1 and 2 of a PH with the default CAN ID.
  private final Solenoid m_Solenoid = new Solenoid(
    PneumaticsModuleType.REVPH,
    0
  );

  // Compressor connected to a PH with a default CAN ID (1)
  private final Compressor m_compressor = new Compressor(
    PneumaticsModuleType.REVPH
  );

  static final int kSolenoidForwardButton = 2;
  static final int kSolenoidReverseButton = 3;
  static final int kCompressorButton = 4;

  static final int kMotor1CANID = 10;
  static final int kMotor2CANID = 20;

  private final SparkMax m_Motor1 = new SparkMax(
    kMotor1CANID,
    MotorType.kBrushless
  );
  private final SparkMax m_Motor2 = new SparkMax(
    kMotor2CANID,
    MotorType.kBrushless
  );

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // Publish elements to shuffleboard.
    ShuffleboardTab tab = Shuffleboard.getTab("Pneumatics");
    tab.add("Solenoid", m_Solenoid);
    tab.add("Compressor", m_compressor);

    // Also publish some raw data
    // Get the pressure (in PSI) from the analog sensor connected to the PH.
    // This function is supported only on the PH!
    // On a PCM, this function will return 0.
    tab.addDouble("PH Pressure [PSI]", m_compressor::getPressure);
    // Get compressor current draw.
    tab.addDouble("Compressor Current", m_compressor::getCurrent);
    // Get whether the compressor is active.
    tab.addBoolean("Compressor Active", m_compressor::isEnabled);
    // Get the digital pressure switch connected to the PCM/PH.
    // The switch is open when the pressure is over ~120 PSI.
    tab.addBoolean("Pressure Switch", m_compressor::getPressureSwitchValue);

    m_Motor1.set(0.0);
    m_Motor2.set(0.0);
  }

  @SuppressWarnings("PMD.UnconditionalIfStatement")
  @Override
  public void teleopPeriodic() {
    /*
     * The joystick Y axis control the speed of motor 1 and the X
     * axis controls motor 2.
     */
    double speed = m_stick.getY();
    m_Motor1.set(speed);

    speed = m_stick.getX();
    m_Motor2.set(speed);

    /*
     * GetRawButtonPressed will only return true once per press.
     * If a button is pressed, set the solenoid to the respective channel.
     */
    if (m_stick.getRawButtonPressed(kSolenoidForwardButton)) {
      m_Solenoid.set(true);
    } else if (m_stick.getRawButtonPressed(kSolenoidReverseButton)) {
      m_Solenoid.set(false);
    }

    // On button press, toggle the compressor.
    if (m_stick.getRawButtonPressed(kCompressorButton)) {
      // Check whether the compressor is currently enabled.
      boolean isCompressorEnabled = m_compressor.isEnabled();
      if (isCompressorEnabled) {
        // Disable closed-loop mode on the compressor.
        m_compressor.disable();
      } else {
        // Enable closed-loop mode based on the analog pressure sensor connected to the PH.
        // The compressor will run while the pressure reported by the sensor is in the
        // specified range ([70 PSI, 120 PSI] in this example).
        // Analog mode exists only on the PH! On the PCM, this enables digital control.
        m_compressor.enableAnalog(70, 120);
      }
    }
  }
}
