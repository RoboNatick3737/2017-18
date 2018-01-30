package org.firstinspires.ftc.teamcode.structs;

import hankextensions.structs.Vector2D;

public class VariableVector2D
{
    public static VariableVector2D from(final Vector2D base)
    {
        return VariableVector2D.rectangular(
                new Function()
                {
                    public double value(double input)
                    {
                        return base.x;
                    }
                },
                new Function()
                {
                    public double value(double input)
                    {
                        return base.y;
                    }
                });
    }

    public static VariableVector2D polar(Function mag, Function theta)
    {
        return new VariableVector2D(VariableVectorType.POLAR, mag, theta);
    }

    public static VariableVector2D rectangular(Function x, Function y)
    {
        return new VariableVector2D(VariableVectorType.RECTANGULAR, x, y);
    }

    // Type which this was initialized as.
    private enum VariableVectorType {POLAR, RECTANGULAR}
    private final VariableVectorType type;

    // The components of this function.
    private final Function a, b;

    private VariableVector2D(VariableVectorType type, Function a, Function b)
    {
        this.type = type;

        this.a = a;
        this.b = b;
    }

    public Vector2D getVector(double param)
    {
        return type == VariableVectorType.POLAR ?
                Vector2D.polar(a.value(param), b.value(param)) :
                Vector2D.rectangular(a.value(param), b.value(param));
    }
}
