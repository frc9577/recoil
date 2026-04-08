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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
  private double m_targetSpeedrpm = 0.0;
  private boolean m_HasFuel = false;
  private int m_tickCount = 0;
  private boolean m_configValid = false;

  private TalonFX m_motorLeader;
  private TalonFX m_motorFollower;
  private TalonFXConfiguration m_MotorConfigs = new TalonFXConfiguration();

  private double m_P = LauncherConstants.kP;
  private double m_I = LauncherConstants.kI;
  private double m_D = LauncherConstants.kD;
  private double m_MMAccel = LauncherConstants.kMotionMagicAcceleration;
  private double m_MMJerk = LauncherConstants.kMotionMagicJerk;

  private final DigitalInput m_Sensor = new DigitalInput(LauncherConstants.kUpperFuelSensorChannel);

  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, use slot 0 */
  private final MotionMagicVelocityVoltage m_velocityVoltage = new MotionMagicVelocityVoltage(0).withSlot(0);

  /** Creates a new LauncherSubsystem. 
     * @throws Exception */
    public LauncherSubsystem() throws Exception {

    // Create SmartDashboard items first so that these are populated
    // regardless of whether or not the subsystem is present.
    SmartDashboard.putNumber("Launcher RPM", 0.0 );
    SmartDashboard.putNumber("Launcher RPS", 0.0);
    SmartDashboard.putBoolean("Launcher Has Fuel", m_HasFuel);

    // Test mode controls
    SmartDashboard.putNumber("Launcher TestRPM", 0.0 );
    SmartDashboard.putNumber("Launcher TestP", m_P );
    SmartDashboard.putNumber("Launcher TestI", m_I );
    SmartDashboard.putNumber("Launcher TestD", m_D );
    SmartDashboard.putNumber("Launcher TestMMAccel", m_MMAccel );
    SmartDashboard.putNumber("Launcher TestMMJerk", m_MMJerk );
      
    m_motorLeader   = new TalonFX(LauncherConstants.kLauncherFlywheelMotor1CANID);
    m_motorFollower = new TalonFX(LauncherConstants.kLauncherFlywheelMotor2CANID);

    // Check that the launcher motors exist and throw an exception if they don't.
    if(!m_motorLeader.isConnected() || !m_motorFollower.isConnected())
    {
      throw new Exception("At least one launcher motor is not present!");
    }
    
    setMotorPIDConfigs(m_MotorConfigs);

    // Set coast mode so that the flywheel doesn't slam to a halt when the 
    // motors stop.
    m_MotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Set the basic rotation direction for both motors. Note that this setting
    // will be ignored by the follower since we explicitly tell it whether to
    // run in the same direction as or the opposite direction from the leader
    // when we set up the follow relationship below.
    m_MotorConfigs.MotorOutput.Inverted = LauncherConstants.kLauncherMotorForwardIsCCW ? 
                                          InvertedValue.CounterClockwise_Positive :
                                          InvertedValue.Clockwise_Positive;
    applyMotorConfigs(m_MotorConfigs);

    // Set up the follower. Note that setting MotorAlignmentValue tells the system to ignore the
    // follower's MotorOutput.Inverted setting and either run the same direction as, or the opposite
    // direction from, the leader.
    m_motorFollower.setControl(new Follower(m_motorLeader.getDeviceID(), 
                   LauncherConstants.kMotorsDriveInOppositeDirections ? 
                    MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    m_motorLeader.set(0.0);

    m_configValid = true;
  }

  public void applyMotorConfigs(TalonFXConfiguration configs)
  {
    /* Retry config apply up to 5 times, report if failure */
    StatusCode statusLeader = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusLeader = m_motorLeader.getConfigurator().apply(configs);
      if (statusLeader.isOK()) break;
    }
    if (!statusLeader.isOK()) {
      System.out.println("Could not apply configs 1, error code: " + statusLeader.toString());
    }

    StatusCode statusFollower = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      statusFollower = m_motorFollower.getConfigurator().apply(configs);
      if (statusFollower.isOK()) break;
    }
    if (!statusFollower.isOK()) {
      System.out.println("Could not apply configs 2, error code: " + statusFollower.toString());
    }
  }

  public void setMotorPIDConfigs(TalonFXConfiguration configs)
  {
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = LauncherConstants.kS; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = LauncherConstants.kV; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = m_P; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = m_I; // No output for integrated error
    configs.Slot0.kD = m_D; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    var motionMagicConfigs = configs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = m_MMAccel;
    motionMagicConfigs.MotionMagicJerk = m_MMJerk;
  }

  //
  // Set the speed of the launcher flywheel in revolutions per minute.
  //
  public void setTargetSpeedrpm(double RPM)
  {
    m_targetSpeedrpm = RPM;

    // changed from multiplying to dividing 

    double desiredRotationsPerSecond = RPM / 60.0;

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
  // Determine whether or not a fuel is in position beneath the launcher
  // entrance.
  //
  public boolean isFuelAtLauncher()
  {
        boolean sensorRead = m_Sensor.get();
        m_HasFuel = (sensorRead == LauncherConstants.kUpperFuelSensorIsEmpty) ? false : true; 
        return m_HasFuel;
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
        SmartDashboard.putBoolean("Launcher Has Fuel", isFuelAtLauncher());
    }
  }

  public void testPeriodic() {

    double NewP = SmartDashboard.getNumber("Launcher TestP", m_P );
    double NewI = SmartDashboard.getNumber("Launcher TestI", m_I );
    double NewD = SmartDashboard.getNumber("Launcher TestD", m_D );
    double NewMMAccel = SmartDashboard.getNumber("Launcher TestMMAccel", m_MMAccel );
    double NewMMJerk = SmartDashboard.getNumber("Launcher TestMMJerk", m_MMJerk );

    // Did any of the PID tuning parameters change?
    if ((NewP != m_P) || (NewI != m_I) || (NewD != m_D) || (NewMMAccel != m_MMAccel) || (NewMMJerk != m_MMJerk))
    {
      // Yes - something changed. Update the motor configuration to reflect the new tuning parameters.
      m_P = NewP;
      m_I = NewI;
      m_D = NewD;
      m_MMAccel = NewMMAccel;
      m_MMJerk = NewMMJerk;
    
      setMotorPIDConfigs(m_MotorConfigs);
      applyMotorConfigs(m_MotorConfigs);
    }

    double launcherrpm = SmartDashboard.getNumber("Launcher TestRPM", 0.0 );
    this.setTargetSpeedrpm(launcherrpm);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  
  }
}
