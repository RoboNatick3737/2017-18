package org.firstinspires.ftc.teamcode.programs.prelimbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import hankextensions.Core;

/*
Config robot elements(motors and servos)
 */


public abstract class HardwareBase extends Core
{

    //Driving Motors
    protected DcMotor left, right, middle, miniLift, primaryLift;
    protected Servo topLeftGrabber, topRightGrabber, bottomLeftGrabber, bottomRightGrabber, swingServo;


    //Other motors

    @Override
    protected void HARDWARE() throws InterruptedException
    {
        left = initHardwareDevice(DcMotor.class, "Left");
        right = initHardwareDevice(DcMotor.class, "Right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        middle = initHardwareDevice(DcMotor.class, "Middle");

        miniLift = initHardwareDevice(DcMotor.class, "lift 1");
        primaryLift = initHardwareDevice(DcMotor.class, "lift 2");

        topLeftGrabber = initHardwareDevice(Servo.class, "Top Left");
        topRightGrabber = initHardwareDevice(Servo.class, "Top Right");
        bottomLeftGrabber = initHardwareDevice(Servo.class, "Bottom Left");
        bottomRightGrabber = initHardwareDevice(Servo.class, "Bottom Right");

        swingServo = initHardwareDevice(Servo.class, "Swing Servo");
    }
}
