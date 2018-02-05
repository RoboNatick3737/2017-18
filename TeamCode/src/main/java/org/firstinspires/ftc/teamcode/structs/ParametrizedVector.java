package org.firstinspires.ftc.teamcode.structs;

import hankextensions.structs.Vector2D;

/**
 * Looks like multivariable calc was a useful class after all :P
 */
public class ParametrizedVector
{
    public static ParametrizedVector from(final Vector2D base)
    {
        return ParametrizedVector.rectangular(
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

    public static ParametrizedVector polar(Function mag, Function theta)
    {
        return new ParametrizedVector(VariableVectorType.POLAR, mag, theta);
    }

    public static ParametrizedVector rectangular(Function x, Function y)
    {
        return new ParametrizedVector(VariableVectorType.RECTANGULAR, x, y);
    }

    // Type which this was initialized as.
    private enum VariableVectorType {POLAR, RECTANGULAR}
    private final VariableVectorType type;

    // The components of this function.
    private final Function a, b;

    private ParametrizedVector(VariableVectorType type, Function a, Function b)
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
