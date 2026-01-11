// Supports the rotation of the goose arm.
// The coordinate system used here has 0 as the intake down position with angles
// increasing as the arm raises. The assumption is that it starts at a non-zero
// position defined by the constant GooseRotationConstants.kStartingAngle.
// All angles at the interface are in degrees.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GooseRotationConstants;
import frc.robot.commands.ManualRotateCommand;

public class GooseRotationSubsystem extends SubsystemBase {
  private double m_angleSet = 0.0;
  private final CANSparkMax m_rotationMotor = new CANSparkMax(GooseRotationConstants.kRotateMotorCANID, 
                                                              MotorType.kBrushless);
  private SparkAbsoluteEncoder m_Encoder;
  private SparkPIDController m_pidController;

  private double m_encoderGradient;
  private double m_encoderIntercept;

  public double m_kP, m_kI, m_kD, m_kIz, m_kFF, m_kMaxOutput, m_kMinOutput;
                                                        
  /** Creates a new GooseRotationSubsystem. */
  public GooseRotationSubsystem() 
  {
    // Working out values for Gradiant & intercept to support angle to encoder value conversion.
    double angleDelta = GooseRotationConstants.kTopAngle-GooseRotationConstants.kBottomAngle;
    double encoderDelta = GooseRotationConstants.kTopRawEncoderValue-GooseRotationConstants.kBottomRawEncoderValue;
    m_encoderGradient = angleDelta/encoderDelta;
    m_encoderIntercept = GooseRotationConstants.kTopAngle-(m_encoderGradient*GooseRotationConstants.kTopRawEncoderValue);


    m_rotationMotor.restoreFactoryDefaults();
    m_rotationMotor.setInverted(true);
    m_rotationMotor.setSmartCurrentLimit(GooseRotationConstants.kMotorCurrentLimit);
    
    m_Encoder = m_rotationMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    // TODO: We may need to set the current position here or, perhaps, just read the encoder
    // so that we know where our reference is.

    m_pidController = m_rotationMotor.getPIDController();
    m_pidController.setFeedbackDevice(m_Encoder);

    // Set default PID coefficients.
    m_kP = GooseRotationConstants.kP;
    m_kI = GooseRotationConstants.kI;
    m_kD = GooseRotationConstants.kD;
    m_kIz = GooseRotationConstants.kIZone;
    m_kFF = GooseRotationConstants.kFF;
    m_kMaxOutput = GooseRotationConstants.kMaxOutput;
    m_kMinOutput = GooseRotationConstants.kMinOutput;

    // Set PID coefficients
    m_pidController.setP(m_kP);
    m_pidController.setI(m_kI);
    m_pidController.setD(m_kD);
    m_pidController.setIZone(m_kIz);
    m_pidController.setFF(m_kFF);
    m_pidController.setOutputRange(m_kMinOutput, m_kMaxOutput);

    if (GooseRotationConstants.kUseSmartMotion)
    {
      m_pidController.setSmartMotionMaxVelocity(GooseRotationConstants.kMaxVelocity, GooseRotationConstants.kSmartMotionSlot);
      m_pidController.setSmartMotionMinOutputVelocity(GooseRotationConstants.kMinVelocity, GooseRotationConstants.kSmartMotionSlot);
      m_pidController.setSmartMotionMaxAccel(GooseRotationConstants.kMaxAcceleration, GooseRotationConstants.kSmartMotionSlot);
      m_pidController.setSmartMotionAllowedClosedLoopError(GooseRotationConstants.kMaxError, GooseRotationConstants.kSmartMotionSlot);
    }
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", m_kP);
    SmartDashboard.putNumber("I Gain", m_kI);
    SmartDashboard.putNumber("D Gain", m_kD);
    SmartDashboard.putNumber("I Zone", m_kIz);
    SmartDashboard.putNumber("Feed Forward", m_kFF);
    SmartDashboard.putNumber("Max Output", m_kMaxOutput);
    SmartDashboard.putNumber("Min Output", m_kMinOutput);
    SmartDashboard.putNumber("Set Degress", 0);
    SmartDashboard.putNumber("Set Reference", getMotorPositionFromAngle(m_angleSet));
  }

  // Given a position read from the encoder, return the equivalent arm
  // angle.
  private double getAngleFromMotorPosition(double encoderReading)
  {
    double angle = (m_encoderGradient * encoderReading) + m_encoderIntercept;
    return angle;
  }

  // Given a desired arm angle in degrees, return the equivalent setpoint
  // value to pass to the PID controller.
  private double getMotorPositionFromAngle(double angle)
  {
    // y = mx+b (c is for the British but we are in the land of the free *eagle sound here*)
    double encoder = (angle - m_encoderIntercept) / m_encoderGradient;
    return encoder;
  }

  public void setSetPointAngle(double angleDegrees)
  {
    m_angleSet = angleDegrees;
    double setpoint = getMotorPositionFromAngle(angleDegrees);
    SmartDashboard.putNumber("Set Degress", m_angleSet);
    SmartDashboard.putNumber("Set Reference", setpoint);
    if(GooseRotationConstants.kUseSmartMotion)
    {
      m_pidController.setReference(setpoint, CANSparkBase.ControlType.kSmartMotion);
    }
    else
    {
      m_pidController.setReference(setpoint, CANSparkBase.ControlType.kPosition);
    }
  }

  public double getSetPointAngle()
  {
    return m_angleSet;
  }

  public double getAngle()
  {
    return getAngleFromMotorPosition(m_Encoder.getPosition());
  }

  public double getSpeed()
  {
    return m_rotationMotor.get();
  }

  public double getEncoderValue()
  {
    return m_Encoder.getPosition();
  }

  public void initDefaultCommand(XboxController speedJoystick)
  {
    setDefaultCommand(new ManualRotateCommand(this, speedJoystick));
  }

  @Override
  public void periodic()
  {
    // TODO: Remove coefficient reading when we've finished tuning!

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != m_kP)) { m_pidController.setP(p); m_kP = p; }
    if((i != m_kI)) { m_pidController.setI(i); m_kI = i; }
    if((d != m_kD)) { m_pidController.setD(d); m_kD = d; }
    if((iz != m_kIz)) { m_pidController.setIZone(iz); m_kIz = iz; }
    if((ff != m_kFF)) { m_pidController.setFF(ff); m_kFF = ff; }
    if((max != m_kMaxOutput) || (min != m_kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      m_kMinOutput = min; m_kMaxOutput = max; 
    }
  }

}
