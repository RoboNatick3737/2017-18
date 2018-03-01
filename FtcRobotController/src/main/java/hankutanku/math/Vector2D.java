package hankutanku.math;

import java.text.DecimalFormat;

public class Vector2D {
    public static Vector2D polar(double magnitude, Angle angle)
    {
        return new Vector2D(magnitude, angle);
    }

    public static Vector2D rectangular(double x, double y) {
        return new Vector2D(x, y, VectorCoordinates.RECTANGULAR);
    }

    public static Vector2D clone(Vector2D other) {
        return new Vector2D(other.x, other.y, VectorCoordinates.RECTANGULAR);
    }

    public enum VectorCoordinates {
        POLAR,
        RECTANGULAR
    }

    private final DecimalFormat vectorPropertyFormatter = new DecimalFormat("#.00");

    public final static Vector2D ZERO = Vector2D.rectangular(0, 0);

    public final double x, y, magnitude;
    public final Angle angle;

    private Vector2D(double x, double y) {
        this.x = x;
        this.y = y;

        // Calculate unsupplied properties.
        this.magnitude = Math.sqrt(x * x + y * y);
        if (this.magnitude != 0)
            this.angle = Angle.degrees(Math.toDegrees(Math.atan2(y, x)));
        else
            this.angle = Angle.degrees(0);
    }

    private Vector2D(double mag, Angle angle)
    {
        this.magnitude = Math.abs(mag);

        if (mag < 0)
            this.angle = angle.opposing();
        else
            this.angle = angle;

        this.x = magnitude * Math.cos(Math.toRadians(angle.value(Angle.MeasurementType.DEGREES)));
        this.y = magnitude * Math.sin(Math.toRadians(angle.value(Angle.MeasurementType.DEGREES)));
    }

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
                    this.angle = Angle.degrees(Math.toDegrees(Math.atan2(y, x)));
                else
                    this.angle = Angle.degrees(0);

                break;

            case POLAR:
                this.magnitude = Math.abs(val1);

                if (val1 < 0)
                    this.angle = Angle.degrees(val2 - 180);
                else
                    this.angle = Angle.degrees(val2);

                this.x = magnitude * Math.cos(Math.toRadians(angle.value(Angle.MeasurementType.DEGREES)));
                this.y = magnitude * Math.sin(Math.toRadians(angle.value(Angle.MeasurementType.DEGREES)));

                break;

            default: // Just here to make android studio stop complaining.
                this.x = 0;
                this.y = 0;
                this.magnitude = 0;
                this.angle = Angle.ZERO;

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
     * Rotates a vector by some angle.
     * @param rotAngle the vector by which to rotate.
     * @return
     */
    public Vector2D rotateBy(Angle rotAngle)
    {
        return Vector2D.polar(magnitude, angle.add(rotAngle));
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
                return "<" + vectorPropertyFormatter.format(x) + ", " + vectorPropertyFormatter.format(y) + ">";

            case POLAR:
                return "<" + vectorPropertyFormatter.format(magnitude) + ", " + vectorPropertyFormatter.format(angle.value(Angle.MeasurementType.DEGREES)) + ">";
        }

        return ""; // Satisfy android studio
    }
}