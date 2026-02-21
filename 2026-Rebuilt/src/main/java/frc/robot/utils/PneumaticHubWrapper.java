//
// A trivial class that wraps PneumaticHub to allow us to throw an exception
// if the device doesn't exist on the CAN bus.
//
package frc.robot.utils;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.hal.REVPHVersion;

public class PneumaticHubWrapper extends PneumaticHub {
    private PneumaticHub m_Hub;

    /* @throws Exception */
    public PneumaticHubWrapper() throws Exception {
        m_Hub = new PneumaticHub();

        if (!isAvailable())
        {
            throw new Exception("Pneumatic hub is not connected or is malfunctioning.");
        }
    }

    /* @throws Exception */
    public PneumaticHubWrapper(int module) throws Exception {
        m_Hub = new PneumaticHub(module);

        if (!isAvailable())
        {
            throw new Exception("Pneumatic hub is not connected or is malfunctioning.");
        }
    }

    public boolean isAvailable()
    {
        REVPHVersion Version = m_Hub.getVersion();

        return (Version.firmwareMajor == 0) ? false : true;
    }
}
