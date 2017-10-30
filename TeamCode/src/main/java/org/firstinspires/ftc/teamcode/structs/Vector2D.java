package org.firstinspires.ftc.teamcode.structs;

public class Vector2D
{
    public static Vector2D fromPolar(double mag, double theta)
    {
        return new Vector2D(mag * Math.cos(theta * (Math.PI/180.0)), mag * Math.sin(theta * (Math.PI/180.0)));
    }

    public final static Vector2D ZERO = new Vector2D(0, 0);

    public final double x, y;

    public Vector2D(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public double magnitude()
    {
        return Math.sqrt(x * x + y * y);
    }

    public double angle()
    {
        double degree = Math.toDegrees(Math.atan2(y, x));

        // Wrap vector degree
        if (degree >= 360)
            degree -= 360;
        else if (degree < 0)
            degree += 360;

        return degree;
    }

    /*
     * Vector operations
     */
    public Vector2D add(Vector2D other)
    {
        return new Vector2D(this.x + other.x, this.y + other.y);
    }
    public Vector2D multiply(double coefficient)
    {
        return new Vector2D(x * coefficient, y * coefficient);
    }
    public Vector2D subtract(Vector2D other)
    {
        return this.add(other.multiply(-1));
    }
    public Vector2D divide (double coefficient)
    {
        if (coefficient == 0)
            return Vector2D.ZERO;
        return this.multiply(1.0 / coefficient);
    }

    public Vector2D unit()
    {
        return this.divide(magnitude());
    }

    public Vector2D orientToAngle(double newAngle)
    {
        double magnitude = magnitude();
        double angleRadians = Math.toRadians(new LimitAngle(newAngle).value);
        return new Vector2D(magnitude * Math.cos(angleRadians), magnitude * Math.sin(angleRadians));
    }

    public boolean equals(Vector2D other)
    {
        return other.x == x && other.y == y;
    }

    public String toString()
    {
        return "<" + x + ", " + y + ">";
    }
}
