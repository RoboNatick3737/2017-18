package hankutanku.math;

public class Polynomial implements Function
{
    private double[] coefficients;

    public Polynomial(double... coefficients)
    {
        this.coefficients = coefficients;
    }

    public Double value(double input)
    {
        double total = 0;
        for (int i = 0; i < coefficients.length; i++)
            total += coefficients[i] * Math.pow(input, coefficients.length - 1 - i);
        return total;
    }
}
