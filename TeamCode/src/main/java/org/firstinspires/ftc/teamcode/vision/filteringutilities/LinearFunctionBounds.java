package org.firstinspires.ftc.teamcode.vision.filteringutilities;

import org.firstinspires.ftc.teamcode.structs.Linear;

public class LinearFunctionBounds
{
    public final Linear lower, upper;

    public LinearFunctionBounds(Linear lower, Linear upper)
    {
        this.lower = lower;
        this.upper = upper;
    }
}
