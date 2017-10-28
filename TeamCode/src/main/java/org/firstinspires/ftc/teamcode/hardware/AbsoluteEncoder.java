package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

public class AbsoluteEncoder extends AnalogInput //already implements HardwareDevice
{
    /**
     * Constructor
     *
     * @param controller AnalogInput controller this channel is attached to
     * @param channel    channel on the analog input controller
     */
    public AbsoluteEncoder(AnalogInputController controller, int channel) {
        super(controller, channel);
    }

    /// Custom methods start ///

    public double position()
    {
        return getVoltage() / 5.0 * 360;
    }

    /// Custom methods end ///
}
