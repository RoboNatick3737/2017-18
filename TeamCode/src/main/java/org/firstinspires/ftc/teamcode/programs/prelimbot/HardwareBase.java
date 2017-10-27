package org.firstinspires.ftc.teamcode.programs.prelimbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import hankextensions.Core;

/*
Config robot elements(motors and servos)
 */


public abstract class HardwareBase extends Core
{

    //Driving Motors
    protected DcMotor left, right, middle, liftylift1, liftylift2;
    protected Servo servo1, servo2;


    //Other motors

    @Override
    protected void HARDWARE() throws InterruptedException
    {
        left = initHardwareDevice(DcMotor.class, "Left");
        right = initHardwareDevice(DcMotor.class, "Right");
        middle = initHardwareDevice(DcMotor.class, "Middle");
        liftylift1 = initHardwareDevice(DcMotor.class, "LiftyLift1");
        liftylift2 = initHardwareDevice(DcMotor.class, "LiftyLift2");

        servo1 = initHardwareDevice(Servo.class, "Servo1");
        servo2 = initHardwareDevice(Servo.class, "Servo2");
    }
}
