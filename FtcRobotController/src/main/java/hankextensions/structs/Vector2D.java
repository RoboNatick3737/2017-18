package hankextensions.structs;

public class Vector2D
{
    public static double clampAngle(double angle)
    {
        while (angle < 0.0)
            angle += 360.0;
        while (angle >= 360.0)
            angle -= 360.0;

        return angle;
    }

    public static Vector2D polar(double magnitude, double angle)
    {
        return new Vector2D(magnitude, angle, VectorCoordinates.POLAR);
    }

    public static Vector2D rectangular(double x, double y)
    {
        return new Vector2D(x, y, VectorCoordinates.RECTANGULAR);
    }

    public static Vector2D clone(Vector2D other)
    {
        return new Vector2D(other.x, other.y, VectorCoordinates.RECTANGULAR);
    }

    public enum VectorCoordinates {
        POLAR,
        RECTANGULAR
    }

    public final static Vector2D ZERO = Vector2D.rectangular(0, 0);

    public final double x, y, magnitude, angle;

    private Vector2D(double val1, double val2, VectorCoordinates coordinateType)
    {
        switch (coordinateType)
        {
            case RECTANGULAR:
                this.x = val1;
                this.y = val2;

                // Calculate unsupplied properties.
                this.magnitude = Math.sqrt(x * x + y * y);
                if (this.magnitude != 0)
                    this.angle = clampAngle(Math.toDegrees(Math.atan2(y, x)));
                else
                    this.angle = 0;

                break;

            case POLAR:
                this.magnitude = val1;
                this.angle = clampAngle(val2);

                this.x = magnitude * Math.cos(Math.toRadians(angle));
                this.y = magnitude * Math.sin(Math.toRadians(angle));

                break;

            default: // Just here to make android studio stop complaining.
                this.x = 0;
                this.y = 0;
                this.magnitude = 0;
                this.angle = 0;

                break;
        }
    }

    /**
     * Adds this to another vector.
     * @param other
     * @return
     */
    public Vector2D add(Vector2D other)
    {
        return Vector2D.rectangular(this.x + other.x, this.y + other.y);
    }

    /**
     * Multiplies this with another vector.
     * @param coefficient
     * @return
     */
    public Vector2D multiply(double coefficient)
    {
        return Vector2D.rectangular(x * coefficient, y * coefficient);
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
     * Calculates the shortest angle between this and another vector.
     * @param other the other vector.
     * @return the shortest angle between this and the other vector.
     */
    public double leastAngleTo(Vector2D other)
    {
        double diff = (other.angle - angle + 180) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }

    /**
     * Rotates a vector by some angle.
     * @param rotAngle the vector by which to rotate.
     * @return
     */
    public Vector2D rotateBy(double rotAngle)
    {
        return Vector2D.polar(magnitude, angle + clampAngle(rotAngle));
    }

    public String toString()
    {
        return toString(VectorCoordinates.RECTANGULAR);
    }
    public String toString(VectorCoordinates coordinate)
    {
        switch (coordinate)
        {
            case RECTANGULAR:
                return "<" + x + ", " + y + ">";

            case POLAR:
                return "<" + magnitude + ", " + angle + ">";
        }

        return ""; // Satisfy android studio
    }
}