//
// Subsystem offering low level control of the launcher flywheel and
// fuel lift mechanisms.
//

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  private double m_targetSpeedrpm = 0.0;
  private boolean m_liftRunning = false;
  private int m_tickCount = 0;
  private boolean m_configValid = false;

  private TalonFX m_motorLeader;
  private TalonFX m_motorFollower;
  private SparkMax m_motorLift;

  private final DigitalInput m_Sensor = new DigitalInput(LauncherConstants.kUpperFuelSensorChannel);

  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, use slot 0 */
  private final MotionMagicVelocityVoltage m_velocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0);

  /** Creates a new LauncherSubsystem. 
     * @throws Exception */
    public LauncherSubsystem() throws Exception {

    m_motorLift     = new SparkMax(LauncherConstants.kLauncherLiftMotorCANID, MotorType.kBrushless);
    m_motorLeader   = new TalonFX(LauncherConstants.kLauncherFlywheelMotor1CANID);
    m_motorFollower = new TalonFX(LauncherConstants.kLauncherFlywheelMotor2CANID);

    // Check that the launcher motors exist and throw an exception if they don't.
    if(!m_motorLeader.isConnected() || !m_motorFollower.isConnected())
    {
      throw new Exception("At least one launcher motor is not present!");
    }

    // Ensure that the lift motor is present.
    SparkBase.Faults Faults = m_motorLift.getFaults();
    if(Faults.can || Faults.firmware || Faults.gateDriver)
    {
      throw new Exception("Lift motor is not present or is reporting a fault!");
    }

    TalonFXConfiguration configs = new TalonFXConfiguration();
    
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = LauncherConstants.kS; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = LauncherConstants.kV; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = LauncherConstants.kP; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = LauncherConstants.kI; // No output for integrated error
    configs.Slot0.kD = LauncherConstants.kD; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    var motionMagicConfigs = configs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = LauncherConstants.kMotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = LauncherConstants.kMotionMagicJerk;

    // Set coast mode so that the flywheel doesn't slam to a halt when the 
    // motors stop.
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Set the basic rotation direction for both motors. Note that this setting
    // will be ignored by the follower since we explicitly tell it whether to
    // run in the same direction as or the opposite direction from the leader
    // when we set up the follow relationship below.
    configs.MotorOutput.Inverted = LauncherConstants.kLauncherMotorForwardIsCCW ? 
                                      InvertedValue.CounterClockwise_Positive :
                                      InvertedValue.Clockwise_Positive;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motorLeader.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs 1, error code: " + status.toString());
    }
    for (int i = 0; i < 5; ++i) {
      status = m_motorFollower.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs 2, error code: " + status.toString());
    }

    // Set up the follower. Note that setting MotorAlignmentValue tells the system to ignore the
    // follower's MotorOutput.Inverted setting and either run the same direction as, or the opposite
    // direction from, the leader.
    m_motorFollower.setControl(new Follower(m_motorLeader.getDeviceID(), 
                   LauncherConstants.kMotorsDriveInOppositeDirections ? 
                    MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    // Configure the fuel sensor.
    // TODO: Add this.

    m_configValid = true;
  }

  //
  // Set the speed of the launcher flywheel in revolutions per minute.
  //
  public void setTargetSpeedrpm(double RPM)
  {
    m_targetSpeedrpm = RPM;

    double desiredRotationsPerSecond = RPM * 60.0;

    SmartDashboard.putNumber("Launcher Set RPM", RPM);
    SmartDashboard.putNumber("Launcher Set RPS", desiredRotationsPerSecond);

    /* Use velocity voltage */
    m_motorLeader.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
  }

  //
  // Get the current launcher flywheel speed in revolutions per minute.
  //
  public double getCurrentSpeedrpm()
  {
    // Remember to convert motor's revs per second velocity to revs per minute!
    return m_motorLeader.getVelocity().getValueAsDouble() * 60.0;
  }

  //
  // Get the current target speed for the launcher flywheel in revolutions
  // per minute.
  //
  public double getTargetSpeedrpm()
  {
    return m_targetSpeedrpm;
  }

  //
  // Start the lift mechanism motor.
  //
  public void startLift()
  {
    m_motorLift.set(LauncherConstants.kLiftMotorSpeed);
    m_liftRunning = true;
  }

  //
  // Stop the lift mechanism motor.
  //
  public void stopLift()
  {
    m_motorLift.set(0.0);
    m_liftRunning = false;
  }

  // 
  // Determine whether or not the lift motor is running.
  public boolean isLiftMotorStarted()
  {
    return m_liftRunning;
  }

  //
  // Determine whether or not a fuel is in position beneath the launcher
  // entrance.
  //
  public boolean isFuelAtLauncher()
  {
        boolean sensorRead = m_Sensor.get();
        return (sensorRead == LauncherConstants.kUpperFuelSensorIsEmpty) ? false : true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_tickCount++;

    if(m_configValid && ((m_tickCount % LauncherConstants.kTicksPerUpdate) == 0))
    {
        double speedRPS = m_motorLeader.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Launcher RPM", speedRPS * 60.0);
        SmartDashboard.putNumber("Launcher RPS", speedRPS);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
