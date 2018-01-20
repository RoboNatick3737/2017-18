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
        this.rightHarvester.setPosition(1);
        this.leftHarvester.setPosition(-1);
        this.secondaryHarvester.setPower(1);
    }

    public void expel()
    {
        this.rightHarvester.setPosition(-1);
        this.leftHarvester.setPosition(1);
        this.secondaryHarvester.setPower(-1);
    }

    public void stop()
    {
        this.leftHarvester.setPosition(0.5);
        this.rightHarvester.setPosition(0.5);
        this.secondaryHarvester.setPower(0);
    }
}
