package hankutanku.math;

public class Vector2D
{
    public static Vector2D clone(Vector2D other)
    {
        return new Vector2D(other.x, other.y);
    }

    public enum VectorCoordinates {
        POLAR,
        RECTANGULAR
    }

    public final static Vector2D ZERO = new Vector2D(0, 0);

    private Double x = null, y = null, magnitude = null;
    private Angle angle = null;

    public Vector2D (double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public Vector2D (double magnitude, Angle angle)
    {
        this.magnitude = Math.abs(magnitude);
        this.angle = magnitude < 0 ? angle.opposing() : angle;
    }

    public double x()
    {
        if (x == null)
            this.x = magnitude * Math.cos(angle.value(Angle.MeasurementType.RADIANS));

        return x;
    }

    public double y()
    {
        if (y == null)
            this.y = magnitude * Math.sin(angle.value(Angle.MeasurementType.RADIANS));

        return y;
    }

    public double magnitude()
    {
        if (magnitude == null)
            this.magnitude = Math.sqrt(x() * x() + y() * y());

        return magnitude;
    }

    public Angle angle()
    {
        if (angle == null)
            angle = Angle.radians(Math.atan2(y, x));

        return angle;
    }

    /**
     * Adds this to another vector.
     * @param other
     * @return
     */
    public Vector2D add(Vector2D other)
    {
        return new Vector2D(this.x + other.x, this.y + other.y);
    }

    /**
     * Multiplies this with another vector.
     * @param coefficient
     * @return
     */
    public Vector2D multiply(double coefficient)
    {
        return new Vector2D(x * coefficient, y * coefficient);
    }

    /**
     * Subtracts another vector from this.
     * @param other
     * @return
     */
    public Vector2D subtract(Vector2D other)
    {
        return this.add(other.multiply(-1));
    }

    /**
     * Divides this vector by some coefficient.
     * @param coefficient
     * @return
     */
    public Vector2D divide (double coefficient)
    {
        if (coefficient == 0)
            return Vector2D.ZERO;
        return this.multiply(1.0 / coefficient);
    }

    /**
     * Provides the unit vector for this vector.
     * @return
     */
    public Vector2D unit()
    {
        return this.divide(magnitude);
    }

    /**
     * Checks component equality by a threshold (doubles).
     * @param other
     * @return
     */
    public boolean equals(Vector2D other)
    {
        return Math.abs(other.x - x) < .0001 && Math.abs(other.y - y) < .0001;
    }

    /**
     * Rotates a vector by some angle.
     * @param rotAngle the vector by which to rotate.
     * @return
     */
    public Vector2D rotateBy(Angle rotAngle)
    {
        return new Vector2D(magnitude, angle.add(rotAngle));
    }

    public String toString()
    {
        return toString(VectorCoordinates.RECTANGULAR);
    }
    public String toString(VectorCoordinates coordinate)
    {
        return coordinate == VectorCoordinates.RECTANGULAR ?
                "<" + x + ", " + y + ">" :
                "<" + magnitude + ", " + angle + ">";
    }
}