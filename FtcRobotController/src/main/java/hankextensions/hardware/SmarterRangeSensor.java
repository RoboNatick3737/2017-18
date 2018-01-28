package hankextensions.hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

public class SmarterRangeSensor
{
    public final ModernRoboticsI2cRangeSensor rangeSensor;

    private double lastValidReading = 255;

    public SmarterRangeSensor(ModernRoboticsI2cRangeSensor rangeSensor, int i2caddress)
    {
        this.rangeSensor = rangeSensor;
        rangeSensor.setI2cAddress(I2cAddr.create8bit(i2caddress));
        rangeSensor.enableLed(true);
    }

    public double getForwardDist()
    {
        double reading = rangeSensor.cmUltrasonic();
        if (reading < 255)
            lastValidReading = reading;

        return lastValidReading;
    }
}
