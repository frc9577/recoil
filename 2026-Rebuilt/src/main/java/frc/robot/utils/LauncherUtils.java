package frc.robot.utils;

public class LauncherUtils {
    //
    // TODO: This table must be filled in either data determined experimentally
    // on the final robot. The first column is the distance from the hub center
    // and the second is the flywheel RPM required to successfully launch fuel into
    // the hub from that distance. Values must be ordered by distance with the lowest
    // distance value first.
    //
    // This array must have at least 3 rows!
    //
    // The data here is currently completely fabricated just to allow the code to
    // be debugged!!!!
    //
    private static final double[][] m_DistanceToRPM = {
        {1.0, 2000.0},
        {2.0, 2400.0},
        {3.0, 3000.0},
        {4.0, 4000.0},
    };
    //
    // Given a distance between the robot and the hub, calculate the 
    // launcher flywheel speed needed to successfully score fuel from
    // that distance.
    //
    public static double getFlywheelSpeed(double Distancem)
    {
        double target = 0.0;

        // Look for cases where we're outside the range from which we can 
        // successfully score. If we're too close, return the minimum
        // speed.
        if (Distancem <= m_DistanceToRPM[0][0])
        {
            return m_DistanceToRPM[0][1];
        }

        // Are we too far away from the hub to score? If so, return the 
        // maximum speed.
        if (Distancem >= m_DistanceToRPM[m_DistanceToRPM.length - 1][0])
        {
            return m_DistanceToRPM[m_DistanceToRPM.length - 1][1];
        }

        // Only scan the table if the current distance is higher than
        // the minimum from which we can score. In cases where we're closer
        // than this, return the minimum launcher speed.
        if (Distancem > m_DistanceToRPM[0][0])
        {
            for (int i = 0; i < (m_DistanceToRPM.length - 1); i++)
            {
                if((Distancem > m_DistanceToRPM[i][0]) && (Distancem <= m_DistanceToRPM[i+1][0]))
                {
                    // We found the interval containing the requested distance so determine
                    // the required target speed using linear interpolation.
                    target = m_DistanceToRPM[i][1] +
                       (((m_DistanceToRPM[i+1][1] - m_DistanceToRPM[i][1]) /
                        (m_DistanceToRPM[i+1][0] - m_DistanceToRPM[i][0])) *
                        (Distancem - m_DistanceToRPM[i][0]));

                    return target;
                }
            } 
        }

        // We should never get here but, if we do, return the maximum speed.
        return m_DistanceToRPM[m_DistanceToRPM.length - 1][1];
    }
}
