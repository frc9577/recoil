// TODO: Class header comment.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
    private final Solenoid m_Solenoid1 = new Solenoid(
        PneumaticsModuleType.REVPH,
        AlgaeConstants.kSolenoidChannel1
    );
    private final Solenoid m_Solenoid2 = new Solenoid(
        PneumaticsModuleType.REVPH,
        AlgaeConstants.kSolenoidChannel2
    );

    private boolean m_isExtented = false;

    private final DigitalInput m_Sensor = new DigitalInput(AlgaeConstants.kSensorChannel);

    private double m_MotorSpeed = 0;
    private final SparkMax m_Motor = new SparkMax(AlgaeConstants.kMotorCANID, MotorType.kBrushless);

    /** Creates a new AlgaeSubsystem. */
    public AlgaeSubsystem() 
    {
        // We need to know if the motor controller we need is
        // actually present on the CAN bus and, unfortunately, the 
        // constructor doesn't seem to throw an exception in this case. 
        // Let's query for firmare error status and use this for now.
        // Note that, for some reason, we can't rely on the CAN fault in
        // this case. Go figure.

        if (m_Motor.getFaults().firmware)
        {
            throw new RuntimeException("Algae subsystem motor not present");
        }

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(AlgaeConstants.kMotorCurrentLimit);
        
        m_Motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Sets the position (state) of the mechanism.
    public void setPosition(Boolean isExtended)
    {
        m_isExtented = isExtended;
        
        m_Solenoid1.set(isExtended);
        m_Solenoid2.set(isExtended);
    }

    // gets the last commanded position (state) of the mechanism
    public Boolean isExtended()
    {
        return m_isExtented;
    }

    // checks if you have an algae in posession
    public boolean hasAlgae()
    {
        boolean sensorRead = m_Sensor.get();
        return AlgaeConstants.kSensorFalseIsEmpty ? sensorRead : !sensorRead;
    }

    // sets the speed of the intake
    public void setIntakeSpeed(double speed) {
        m_Motor.set(speed);
        m_MotorSpeed = speed;
    }

    // gets the last commanded speed for the intake
    public double getIntakeSpeed() {
        return m_MotorSpeed;
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
