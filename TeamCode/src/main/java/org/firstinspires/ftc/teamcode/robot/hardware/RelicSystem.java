package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RelicSystem
{
    private final DcMotor extender;
    private final Servo rotator, grabber;

    public RelicSystem(DcMotor extender, Servo rotator, Servo grabber)
    {
        this.extender = extender;
        this.rotator = rotator;
        this.grabber = grabber;

        setGrabber(false);
        stopRotator();
        stopExtender();
    }

    public void extend()
    {
        extender.setPower(1);
    }

    public void retract()
    {
        extender.setPower(-1);
    }

    public void stopExtender()
    {
        extender.setPower(0);
    }

    public void variableExtension(double forward, double backward)
    {
        if (Math.abs(forward - backward) < .05)
            extender.setPower(0);

        extender.setPower(forward - backward);
    }

    public void rotate(boolean forward)
    {
        rotator.setPosition(forward ? 0 : 1);
    }

    public void stopRotator()
    {
        rotator.setPosition(0.5);
    }

    private boolean grabbed = false;
    public void setGrabber(boolean grab)
    {
        grabber.setPosition(grab ? 0 : 1);
        grabbed = grab;
    }
    public void toggleGrabber()
    {
        setGrabber(!grabbed);
    }
}
