package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

public class LauncherUtils {
    //
    // Characterization data for the launcher. This table links shooting
    // distance to known-good launcher speeds. Distances are from the centre
    // of the robot base to the centre of the hub.
    //
    private static final double[][] m_DistanceToRPM = {
        {2.44, 2470.0},
        {3.12, 2700.0},
        {3.81, 2800.0},
        {5.21, 3300.0},
    };

    //
    // Return the minimum robot-to-hub distance from which we know we
    // can score fuel.
    //
    public static double getMinShootingDistance() {
        return m_DistanceToRPM[0][0];
    }

    public static Boolean canScoreFromHere(DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed) {
        // How far are we from our alliance hub?
        double distance = HubUtils.getHubDistance(poseEstimator, isRed);

        // This doesn't take into consideration which part of the field we are in so will 
        // report true even in the neutral zone. We assume the driver is aware enough of the
        // robot position to know this!
        return ((distance >= m_DistanceToRPM[0][0]) && (distance <= m_DistanceToRPM[m_DistanceToRPM.length - 1][0]));
    }
    
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
