package hankextensions.structs;

import android.support.annotation.NonNull;

public class Angle implements Comparable<Angle>
{
    public static Angle degrees(double degrees)
    {
        while (degrees < 0.0)
            degrees += 360.0;
        while (degrees >= 360.0)
            degrees -= 360.0;

        return new Angle(MeasurementType.DEGREES, degrees);
    }
    public static Angle radians(double radians)
    {
        while (radians < 0.0)
            radians += 2 * Math.PI;
        while (radians >= 2 * Math.PI)
            radians -= 2 * Math.PI;

        return new Angle(MeasurementType.RADIANS, radians);
    }
    public final static Angle ZERO = new Angle(MeasurementType.DEGREES, 0);

    private double degrees = -1, radians = -1;

    public enum MeasurementType {DEGREES, RADIANS}

    private Angle(MeasurementType type, double value)
    {
        switch (type)
        {
            case DEGREES:
                this.degrees = value;
                break;

            case RADIANS:
                this.radians = value;
                break;
        }
    }

    public double value(MeasurementType measurementType)
    {
        if (measurementType == MeasurementType.RADIANS)
        {
            if (radians < 0)
                radians = Math.toRadians(degrees);

            return radians;
        }
        else
        {
            if (degrees < 0)
                degrees = Math.toDegrees(radians);

            return degrees;
        }
    }

    public Angle add(Angle other)
    {
        return Angle.degrees(other.value(MeasurementType.DEGREES) + value(MeasurementType.DEGREES));
    }

    public Angle subtract(Angle other)
    {
        return Angle.degrees(value(MeasurementType.DEGREES) - other.value(MeasurementType.DEGREES));
    }

    public Angle shortestPathTo(Angle other)
    {
        Angle diff = this.subtract(other);
        return diff.value(MeasurementType.DEGREES) > 180 ? Angle.degrees(180 - diff.value(MeasurementType.DEGREES)) : diff;
    }

    public Angle opposing()
    {
        return Angle.degrees(value(MeasurementType.DEGREES) + 180);
    }

    public Angle negative()
    {
        return Angle.degrees(-value(MeasurementType.DEGREES));
    }

    @Override
    public int compareTo(@NonNull Angle another)
    {
        if (value(MeasurementType.DEGREES) < another.value(MeasurementType.DEGREES))
            return -1;
        else if (value(MeasurementType.DEGREES) > another.value(MeasurementType.DEGREES))
            return 1;
        else
            return 0;
    }
}
