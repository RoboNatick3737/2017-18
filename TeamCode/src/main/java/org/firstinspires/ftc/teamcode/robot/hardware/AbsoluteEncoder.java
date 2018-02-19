package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import hankextensions.structs.Angle;

public class AbsoluteEncoder
{
    public final AnalogInput device;

    public AbsoluteEncoder(AnalogInput device) {
        this.device = device;
    }

    public Angle position()
    {
        return Angle.degrees(device.getVoltage() / 5.0 * 360);
    }
}
