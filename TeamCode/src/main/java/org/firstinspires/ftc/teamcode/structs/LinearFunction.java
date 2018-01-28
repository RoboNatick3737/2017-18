package org.firstinspires.ftc.teamcode.structs;

public class LinearFunction implements Function
{
    public final double a, b;

    public LinearFunction(double a, double b)
    {
        this.a = a;
        this.b = b;
    }

    public double value(double input)
    {
        return a * input + b;
    }

    public ComplexFunction toComplexFunction()
    {
        return new ComplexFunction(ComplexFunction.FunctionType.POLYNOMIAL, a, b);
    }
}
