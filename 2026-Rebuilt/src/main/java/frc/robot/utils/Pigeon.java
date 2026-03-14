// Taken and modified from CTRE's example
// https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/d70cab6060617bbed5e207c2eaf8747af09a15f6/Java%20General/Pigeon2/src/main/java/frc/robot/subsystems/PigeonSubsystem.java
package frc.robot.utils;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.RobotConstants;

public class Pigeon {
    private final Pigeon2 m_pigeon2;
    private boolean m_exists;
    private double m_yawOffset = 0;

    public Pigeon(int PigeonCanId) {
        m_pigeon2 = new Pigeon2(PigeonCanId);
        m_exists = m_pigeon2.isConnected();

        if (m_exists == false) {
            DriverStation.reportWarning("Pigeon on canid2 does not exist! Default values will be returned.", true);
        } else {
            Pigeon2Configuration config = new Pigeon2Configuration();
            config.MountPose.MountPoseYaw = RobotConstants.kPigeonYawOffset;
            config.MountPose.MountPosePitch = RobotConstants.kPigeonPitchOffset;
            config.MountPose.MountPoseRoll = RobotConstants.kPigeonRollOffset;
            m_pigeon2.getConfigurator().apply(config);

            m_pigeon2.reset();
        }
    }
  
    private void dosntExist() {
        if (RobotConstants.kDoPigeonWarn) {
            DriverStation.reportWarning("Pigeon on canid 2 does not exist, returning default value!", false);
        }
    }

    public void reset() {
        if (m_exists) {
            m_pigeon2.reset();
        } else {
            dosntExist();
        }
    }

    public Rotation2d getRotation2d() {
        if (m_exists) {
            return m_pigeon2.getRotation2d().rotateBy(new Rotation2d(m_yawOffset * (Math.PI/180)));
        } else {
            dosntExist();
            return Rotation2d.kZero;
        }
    }

    public double getYaw() {
        if (m_exists) {
            return m_pigeon2.getYaw().getValueAsDouble() + m_yawOffset;
        } else {
            dosntExist();
            return 0.0;
        }
    }

    // I dont think offset is needed for this.
    public double getYawRate() {
        if (m_exists) {
            return m_pigeon2.getAngularVelocityZWorld().getValueAsDouble();
        } else {
            dosntExist();
            return 0.0;
        }
    }

    public double getPitch() { 
        if (m_exists) {
            return m_pigeon2.getPitch().getValueAsDouble(); 
        } else {
            dosntExist();
            return 0.0;
        }
    }

    public double getRoll() { 
        if (m_exists) {
            return m_pigeon2.getRoll().getValueAsDouble();
        } else {
            dosntExist();
            return 0.0;
        }
    }
    
    public void setYaw(double yaw) { 
        if (m_exists) {
            m_pigeon2.setYaw(yaw-m_yawOffset); 
        } else {
            dosntExist();
        }
    }
    
    public double getUpTime() { 
        if (m_exists) {
            return m_pigeon2.getUpTime().getValueAsDouble();  
        } else {
            dosntExist();
            return -1;
        } 
    }

    public double getTemp() {
        if (m_exists) {
            return m_pigeon2.getTemperature().getValueAsDouble();  
        } else {
            dosntExist();
            return -1;
        }
    }

    public void setYawOffset(double degrees) {
        m_yawOffset = degrees;
    }

    public double getYawOffset() {
        return m_yawOffset;
    }
}
