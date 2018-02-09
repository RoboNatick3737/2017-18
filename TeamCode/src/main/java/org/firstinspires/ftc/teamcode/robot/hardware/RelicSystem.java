package org.firstinspires.ftc.teamcode.robot.hardware;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RelicSystem
{
    private final DcMotor extender;
    private final Servo grabber;

    public double extensionEstimate = 0;

    public RelicSystem(DcMotor extender, Servo grabber)
    {
        this.extender = extender;
        this.grabber = grabber;

        setGrabber(false);
        stopExtender();
    }

    private ProcessConsole processConsole = null;
    public void setEnableLogging(boolean enableLogging)
    {
        if (enableLogging)
            processConsole = LoggingBase.instance.newProcessConsole("Relic Arm Log");
        else
            processConsole = null;
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
        {
            extender.setPower(0);
            return;
        }

        extensionEstimate += forward - backward;

        if (processConsole != null)
            processConsole.write("Extension is " + extensionEstimate);

        extender.setPower(forward - backward);
    }

    private boolean grabbed = false;
    public void setGrabber(boolean grab)
    {
        grabber.setPosition(grab ? 1 : 0);
        grabbed = grab;
    }
    public void toggleGrabber()
    {
        setGrabber(!grabbed);
    }
}
