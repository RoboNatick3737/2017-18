package org.firstinspires.ftc.teamcode.components;

public class Vector2D
{
    public static Vector2D fromPolar(double mag, double theta)
    {
        return new Vector2D(mag * Math.cos(theta * (Math.PI/180.0)), mag * Math.sin(theta * (Math.PI/180.0)));
    }

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
        return Math.atan2(y, x) * 180.0/Math.PI;
    }

    public Vector2D add(Vector2D other)
    {
        return new Vector2D(this.x + other.x, this.y + other.y);
    }

    public Vector2D unit()
    {
        return new Vector2D(this.x / magnitude(), this.y / magnitude());
    }
}
