package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AbsoluteEncoder
{
    public final AnalogInput device;

    public AbsoluteEncoder(AnalogInput device) {
        this.device = device;
    }

    public double position()
    {
        return device.getVoltage() / 5.0 * 360;
    }
}
