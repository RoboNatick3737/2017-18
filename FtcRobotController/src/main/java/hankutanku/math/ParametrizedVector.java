package hankutanku.math;

import android.support.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Func;

public class ParametrizedVector
{
    public static ParametrizedVector polar(@NonNull Function<Double> magnitude, @NonNull Function<Angle> angle)
    {
        return new ParametrizedVector(null, null, magnitude, angle);
    }

    public static ParametrizedVector rectangular(@NonNull Function<Double> x, @NonNull Function<Double> y)
    {
        return new ParametrizedVector(x, y, null, null);
    }

    public static ParametrizedVector constant(@NonNull final Vector2D vector)
    {
        return new ParametrizedVector(
                new Function<Double>() {
                    @Override
                    public Double value(double input) {
                        return vector.x();
                    }
                },
                new Function<Double>() {
                    @Override
                    public Double value(double input) {
                        return vector.y();
                    }
                },
                null,
                null
        );
    }

    private final Function<Double> x, y;

    private final Function<Double> magnitude;
    private final Function<Angle> angle;

    public ParametrizedVector(Function<Double> x, Function<Double> y, Function<Double> magnitude, Function<Angle> angle)
    {
        this.x = x;
        this.y = y;

        this.magnitude = magnitude;
        this.angle = angle;
    }

    public Vector2D getVector(double param)
    {
        if (x == null)
            return new Vector2D(magnitude.value(param), angle.value(param));
        else
            return new Vector2D(x.value(param), y.value(param));
    }
}
