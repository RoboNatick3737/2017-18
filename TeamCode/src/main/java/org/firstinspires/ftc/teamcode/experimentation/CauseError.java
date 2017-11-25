package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

@Autonomous(name="Cause Error", group= Constants.EXPERIMENTATION)
public class CauseError extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor thing = (DcMotor)(hardwareMap.get("fucku"));
    }
}
