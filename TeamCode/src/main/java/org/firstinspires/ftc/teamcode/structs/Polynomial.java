package org.firstinspires.ftc.teamcode.structs;

public class Polynomial implements Function
{
    private double[] coefficients;

    public Polynomial(double... coefficients)
    {
        this.coefficients = coefficients;
    }

    public double value(double input)
    {
        double total = 0;
        for (int i = 0; i < coefficients.length; i++)
            total += coefficients[i] * Math.pow(input, coefficients.length - 1 - i);
        return total;
    }
}
