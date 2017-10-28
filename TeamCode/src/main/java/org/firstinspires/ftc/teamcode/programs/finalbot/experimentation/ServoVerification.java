package org.firstinspires.ftc.teamcode.programs.finalbot.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.finalbot.HardwareBase;
import org.firstinspires.ftc.teamcode.structs.Vector2D;

import hankextensions.Core;
import hankextensions.threading.Flow;

@Autonomous(name="Servo Verification", group=Constants.FINAL_BOT_EXPERIMENTATION)
public class ServoVerification extends Core
{
    @Override
    protected void START() throws InterruptedException
    {
        Servo test = initHardwareDevice(Servo.class, "Front Right Vex Motor");

        test.setPosition(0);
        log.lines("This is 0");
        Flow.msPause(3000);

        test.setPosition(0.5);
        log.lines("This is 0.5");
        Flow.msPause(3000);

        test.setPosition(1);
        log.lines("This is 1");
        Flow.msPause(3000);
    }
}
