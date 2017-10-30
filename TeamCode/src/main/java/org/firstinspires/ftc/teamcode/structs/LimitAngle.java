package org.firstinspires.ftc.teamcode.structs;

public class LimitAngle
{
    public final double value;

    public LimitAngle(double value)
    {
        // Wrap angle to [-inf,359]
        while (value >= 360)
            value -= 360;

        // Wrap angle to [0,359]
        while (value < 0)
            value += 360;

        this.value = value;
    }

    /**
     * Finds the least angle between this angle and another limit angle.
     */
    public double angleTo(LimitAngle other)
    {
        double diff = (other.value - value + 180) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }

    public LimitAngle add(LimitAngle other)
    {
        double addition = value + this.value;
        if (addition >= 360)
            addition -= 360;

        return new LimitAngle(addition);
    }

    public LimitAngle subtract(LimitAngle other)
    {
        double subtraction = value + this.value;
        if (subtraction < 0)
            subtraction += 360;

        return new LimitAngle(subtraction);
    }
}
