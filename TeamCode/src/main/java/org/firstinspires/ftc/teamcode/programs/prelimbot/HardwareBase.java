package org.firstinspires.ftc.teamcode.programs.prelimbot;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
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
    protected ColorSensor colorSensor;

    private final double OPEN_LEFT_POS   = .7;
    private final double OPEN_RIGHT_POS  = .2;
    private final double CLOSE_LEFT_POS  = .2;
    private final double CLOSE_RIGHT_POS = .7;

    //Other motors

    protected void closeClamps() {
        topLeftGrabber.    setPosition(CLOSE_LEFT_POS);
        topRightGrabber.   setPosition(CLOSE_RIGHT_POS);
        bottomLeftGrabber. setPosition(CLOSE_LEFT_POS);
        bottomRightGrabber.setPosition(CLOSE_RIGHT_POS);
    }

    protected void openClamps() {
        topLeftGrabber.    setPosition(OPEN_LEFT_POS);
        topRightGrabber.   setPosition(OPEN_RIGHT_POS);
        bottomLeftGrabber. setPosition(OPEN_LEFT_POS);
        bottomRightGrabber.setPosition(OPEN_RIGHT_POS);
    }

    @Override
    protected void HARDWARE() throws InterruptedException
    {
        left               = initHardwareDevice(DcMotor.class, "Left");
        right              = initHardwareDevice(DcMotor.class, "Right");
        middle             = initHardwareDevice(DcMotor.class, "Middle");

        miniLift           = initHardwareDevice(DcMotor.class, "lift 1");
        primaryLift        = initHardwareDevice(DcMotor.class, "lift 2");

        topLeftGrabber     = initHardwareDevice(Servo.class, "Top Left");
        topRightGrabber    = initHardwareDevice(Servo.class, "Top Right");
        bottomLeftGrabber  = initHardwareDevice(Servo.class, "Bottom Left");
        bottomRightGrabber = initHardwareDevice(Servo.class, "Bottom Right");

        swingServo = initHardwareDevice(Servo.class, "Swing Servo");

        colorSensor = initHardwareDevice(ColorSensor.class, "Color Sensor");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
