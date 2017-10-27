package org.firstinspires.ftc.teamcode.programs.prelimbot.programs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.programs.prelimbot.HardwareBase;

import hankextensions.threading.Flow;

@TeleOp(name="Move Their Servo", group="Experimentation")
public class MoveTheirServo extends HardwareBase
{
    private Servo servo1;
    private Servo servo2;
    @Override
    protected void INITIALIZE() throws InterruptedException
    {
        servo1 = initHardwareDevice(Servo.class, "Servo1");
        servo2 = initHardwareDevice(Servo.class, "Servo2");
    }

    @Override
    protected void START() throws InterruptedException
    {
        while (true)
        {
            if (gamepad1.a)
            {
                servo1.setPosition(1);
                servo2.setPosition(-1);
            }
            else
            {
                servo1.setPosition(-1);
                servo2.setPosition(1);
            }

            Flow.yield();
        }
    }
}
