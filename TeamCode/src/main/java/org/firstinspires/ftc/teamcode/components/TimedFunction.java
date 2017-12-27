package org.firstinspires.ftc.teamcode.components;

/**
 * Represents a movement that a motor, servo, or larger system can accomplish, with a variety
 * of mathematical formulae.
 */
public class TimedFunction
{
    public enum Type { CONSTANT, LINEAR, LOGARITHMIC, PARABOLIC, EXPONENTIAL }
    private final Type TYPE;

    private final long START_TIME;
    private int[] components = null;

    // Throws error if insufficient
    private void ensureMovementParamsSufficient(Type type, int[] additionalParams)
    {
        int required = 0;

        switch (type)
        {
            case CONSTANT:
                required = 1; // y = a
                break;

            case LINEAR:
                required = 2; // y = a * x + b
                break;

            case LOGARITHMIC:
                required = 3; // y = a * log(b * x) + c
                break;

            case PARABOLIC:
                required = 3; // y = a * x^2 + b * x + c
                break;

            case EXPONENTIAL:
                required = 3; // y = a * e^(b * x) + c
                break;
        }

        if (additionalParams == null || additionalParams.length < required)
            throw new Error(type + " requires " + required + " parameters.");
    }

    public TimedFunction(Type type, int... additionalParams)
    {
        ensureMovementParamsSufficient(type, additionalParams); // throws an error if incorrect number of params

        this.TYPE = type;
        this.components = additionalParams;

        START_TIME = System.currentTimeMillis();
    }

    // Returns the function's value at this point in elapsed time.
    public double value()
    {
        double elapsed = System.currentTimeMillis() - START_TIME;

        switch (TYPE)
        {
            case CONSTANT:
                return components[0];

            case LINEAR:
                return components[0] * elapsed + components[1];

            case PARABOLIC:
                return components[0] * elapsed * elapsed + components[1] * elapsed + components[2];

            case EXPONENTIAL:
                return components[0] * Math.pow(Math.E, components[1]) + components[2];

            case LOGARITHMIC:
                return components[0] * Math.log(components[1]) + components[2];
        }

        return 0; // does nothing just here to satisfy android studio
    }
}
