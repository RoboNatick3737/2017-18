package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The left and right flippers for the glyphs.
 */
public class Intake
{
    private final Servo leftHarvester, rightHarvester;
    private final DcMotor secondaryHarvester;

    public Intake(Servo leftHarvester, Servo rightHarvester, DcMotor secondaryHarvester)
    {
        this.leftHarvester = leftHarvester;
        this.rightHarvester = rightHarvester;
        this.secondaryHarvester = secondaryHarvester;

        stop();
    }

    public void intake()
    {
        leftIntake(1);
        rightIntake(1);
        secondaryIntake(1);
    }

    public void expel()
    {
        leftIntake(-1);
        rightIntake(-1);
        secondaryIntake(-1);
    }

    public void stop()
    {
        this.leftHarvester.setPosition(0.5);
        this.rightHarvester.setPosition(0.5);
        this.secondaryHarvester.setPower(0);
    }

    public void leftIntake(double speed)
    {
        if (Math.abs(speed) < .03)
            speed = 0;

        this.leftHarvester.setPosition(0.5 * -speed + 0.5);
    }

    public void rightIntake(double speed)
    {
        if (Math.abs(speed) < .03)
            speed = 0;

        this.rightHarvester.setPosition(0.5 * speed + 0.5);
    }

    public void secondaryIntake(double speed)
    {
        if (Math.abs(speed) < .03)
            speed = 0;

        this.secondaryHarvester.setPower(speed);
    }
}
