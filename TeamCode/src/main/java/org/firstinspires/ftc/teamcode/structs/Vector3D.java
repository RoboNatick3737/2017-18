package org.firstinspires.ftc.teamcode.structs;

public class Vector3D
{
    public static Vector3D fromVector2(Vector2D vector2D)
    {
        return new Vector3D(vector2D.x, vector2D.y, 0);
    }

    public final static Vector3D ZERO = new Vector3D(0, 0, 0);

    public final double x, y, z;

    public Vector3D(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}
