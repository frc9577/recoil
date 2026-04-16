package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.RobotController;

public class CachedBoolean {
    private final BooleanSupplier m_supplier;
    private final long m_decayTimeMicros;

    private long m_timestapMicros;
    private boolean m_cachedValue;

    public CachedBoolean(long decayTimeSecs, BooleanSupplier supplier) {
        m_supplier = supplier;
        m_decayTimeMicros = decayTimeSecs * 1000000;

        m_cachedValue = m_supplier.getAsBoolean();
        m_timestapMicros = RobotController.getFPGATime();
    }

    public Boolean get() {
        long currentTime = RobotController.getFPGATime();
        if ((currentTime - m_timestapMicros) <= m_decayTimeMicros) {
            m_cachedValue = m_supplier.getAsBoolean();
            m_timestapMicros = currentTime;
        }

        return m_cachedValue;
    }
}