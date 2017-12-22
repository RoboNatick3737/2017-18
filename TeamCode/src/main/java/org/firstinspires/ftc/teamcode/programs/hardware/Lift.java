package org.firstinspires.ftc.teamcode.programs.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The left and right flippers for the glyphs.
 */
public class Lift
{
    private final double UP_SPEED = 1, DOWN_SPEED = -1;

    private final DcMotor controller;
    public Lift(DcMotor controller)
    {
        this.controller = controller;
    }

    public void up()
    {
        this.controller.setPower(UP_SPEED);
    }

    public void down()
    {
        this.controller.setPower(DOWN_SPEED);
    }

    public void stop()
    {
        this.controller.setPower(0);
    }

    public void variable(double up, double down)
    {
        this.controller.setPower(UP_SPEED * up + DOWN_SPEED * down);
    }
}
