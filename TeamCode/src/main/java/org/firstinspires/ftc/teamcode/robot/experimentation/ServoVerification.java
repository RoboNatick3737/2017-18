package org.firstinspires.ftc.teamcode.robot.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import hankextensions.RobotCore;

@Autonomous(name="Servo Verification", group=Constants.FINAL_BOT_EXPERIMENTATION)
public class ServoVerification extends RobotCore
{
    @Override
    protected void onRun() throws InterruptedException
    {
        waitForStart();

        Servo test = initHardwareDevice(Servo.class, "Front Right Vex Motor");

        test.setPosition(0);
        log.lines("This is 0");
        flow.msPause(3000);

        test.setPosition(0.5);
        log.lines("This is 0.5");
        flow.msPause(3000);

        test.setPosition(1);
        log.lines("This is 1");
        flow.msPause(3000);
    }
}
