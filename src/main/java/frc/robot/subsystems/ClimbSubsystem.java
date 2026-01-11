// This is the class provides methods to control a pnuematics based climb.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants; 

public class ClimbSubsystem extends SubsystemBase {
 private final DoubleSolenoid m_ClimbSolenoidL = new DoubleSolenoid(
                  ClimbConstants.kPneumaticsHubCANid, PneumaticsModuleType.REVPH, 
                  ClimbConstants.kExtendChannelL, ClimbConstants.kRetractChannelL);
 private final DoubleSolenoid m_ClimbSolenoidR = new DoubleSolenoid(
                  ClimbConstants.kPneumaticsHubCANid, PneumaticsModuleType.REVPH, 
                  ClimbConstants.kExtendChannelR, ClimbConstants.kRetractChannelR);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {}

  public enum State {
    OFF,
    LIFTED,
    GROUNDED
  }

  private State m_currentState = State.OFF;

  public void setClimbState(State newState)
  {
    m_currentState = newState;
    DoubleSolenoid.Value newValue = DoubleSolenoid.Value.kOff;

    switch (m_currentState) 
    {
      case OFF:
        newValue = DoubleSolenoid.Value.kOff;
        break;

      case LIFTED:
        newValue = DoubleSolenoid.Value.kReverse;
        break;

      case GROUNDED:
        newValue = DoubleSolenoid.Value.kForward;
        break;
    }

    m_ClimbSolenoidL.set(newValue);
    m_ClimbSolenoidR.set(newValue);
  }

  public State getClimbState()
  {
    return m_currentState;
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
