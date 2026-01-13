package frc.robot.utils;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DrivetrainConstants;

public class PIDControl {
    private TalonFX m_motor;
    private boolean m_positiveMovesForward;
 
    private double m_targetPosition;

    private final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        DrivetrainConstants.maxVelocity, 
        DrivetrainConstants.maxAcceleration
      )
    );

    private PositionVoltage m_positionVoltage;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setPoint = new TrapezoidProfile.State();

    public PIDControl(TalonFX motor, boolean positiveMovesForward) {
      m_motor = motor;
      m_positiveMovesForward = positiveMovesForward;
    }

    public void setTargetPosition(double position) {
      m_targetPosition = position;

      double positionRotations = position * DrivetrainConstants.kDrivetrainGearRatio;
      if (!m_positiveMovesForward) {
        positionRotations = -positionRotations;
      }

      m_goal = new TrapezoidProfile.State(positionRotations, 0);
      m_setPoint = new TrapezoidProfile.State(0, 0);

      m_positionVoltage = new PositionVoltage(0).withSlot(0);

      m_motor.setPosition(0); // zero's motor at the start of the movement
    }

    public void calculatePosition() {
      m_setPoint = m_profile.calculate(0.020, m_setPoint, m_goal);

      m_positionVoltage.Position = m_setPoint.position;
      m_positionVoltage.Velocity = m_setPoint.velocity;
      m_motor.setControl(m_positionVoltage);
    }

    public double getTargetPosition() {
      return m_targetPosition;
    }

    public double getPosition() {
      double motorPosition = m_motor.getPosition().getValueAsDouble() / DrivetrainConstants.kDrivetrainGearRatio;

      if (m_positiveMovesForward) {
        return motorPosition;
      } else {
        return -motorPosition;
      }
    }

}
