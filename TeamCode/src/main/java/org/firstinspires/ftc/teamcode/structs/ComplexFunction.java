package org.firstinspires.ftc.teamcode.structs;

/**
 * Represents a movement that a motor, servo, or larger system can accomplish, with a variety
 * of mathematical formulae.
 */
public class ComplexFunction implements Function
{
    public enum FunctionType { POLYNOMIAL, LOGARITHMIC, EXPONENTIAL }
    private final FunctionType type;
    private final double[] coefficients;

    public ComplexFunction(FunctionType type, double... coefficients)
    {
        this.type = type;
        this.coefficients = coefficients;
    }

    public double value(double x)
    {
        switch (type)
        {
            case POLYNOMIAL:
                double total = 0;
                for (int i = 0; i < coefficients.length; i++)
                    total += coefficients[i] * Math.pow(x, coefficients.length - 1 - i);
                return total;

            case LOGARITHMIC:
                if (coefficients.length < 3)
                    return 0;

                return coefficients[0] * Math.log(coefficients[1] * x) + coefficients[2];

            case EXPONENTIAL:
                if (coefficients.length < 3)
                    return 0;

                return coefficients[0] * Math.pow(Math.E, x * coefficients[1]) + coefficients[2];

            default:
                return 0;
        }
    }
}
