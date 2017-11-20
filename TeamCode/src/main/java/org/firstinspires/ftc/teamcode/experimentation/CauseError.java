package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

import hankextensions.Core;

@Autonomous(name="Cause Error", group= Constants.EXPERIMENTATION)
public class CauseError extends Core
{
    @Override
    protected void START() throws InterruptedException {
        DcMotor thing = initHardwareDevice(DcMotor.class, "Doesntexist");

        thing.setPower(4);
    }
}
