package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

import hankextensions.EnhancedOpMode;

@Autonomous(name="Modern Robotics PID Testing", group= Constants.EXPERIMENTATION)
public class MRPIDTesting extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        DcMotor motor = hardware.initialize(DcMotor.class, "Motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        motor.setPower(0.5);
        ProcessConsole console = log.newProcessConsole("Console");
        while (true)
        {
            console.write("Pos " + motor.getCurrentPosition());
            flow.yield();
        }
    }
}
