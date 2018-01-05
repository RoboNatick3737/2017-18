package org.firstinspires.ftc.teamcode.vision.filteringutilities;

import org.firstinspires.ftc.teamcode.structs.LinearFunction;

public class LinearFunctionBounds
{
    public final LinearFunction lower, upper;

    public LinearFunctionBounds(LinearFunction lower, LinearFunction upper)
    {
        this.lower = lower;
        this.upper = upper;
    }
}
