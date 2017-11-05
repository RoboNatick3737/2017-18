package org.firstinspires.ftc.teamcode.programs.finalbot.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.hardware.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.SwerveWheel;
import org.firstinspires.ftc.teamcode.structs.Vector2D;

import hankextensions.Core;
import hankextensions.threading.Flow;
import hankextensions.threading.SimpleTaskPackage;

@Autonomous(name="Swivel Wheel PID Testing", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class SwivelWheelTesting extends Core
{
    SwerveWheel frontLeft, backLeft, frontRight, backRight;
    SimpleTaskPackage taskPackage;

    @Override
    protected void INITIALIZE() throws InterruptedException {
        // All of the SwerveWheels (which align on independent threads)
        frontLeft = new SwerveWheel(
                "Front Left",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Left")),
                initHardwareDevice(Servo.class, "Front Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Left Vex Encoder")),
                new PIDConstants(0.020, 0.001, 0.0009, 0),
                59.44);

        frontRight = new SwerveWheel(
                "Front Right",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Right")),
                initHardwareDevice(Servo.class, "Front Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Right Vex Encoder")),
                new PIDConstants(0.019, 0.001, 0.0008, 0),
                42.22);

        backLeft = new SwerveWheel(
                "Back Left",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Left")),
                initHardwareDevice(Servo.class, "Back Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Left Vex Encoder")),
                new PIDConstants(0.018, 0.001, 0.0008, 0),
                40.47);

        backRight = new SwerveWheel(
                "Back Right",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Right")),
                initHardwareDevice(Servo.class, "Back Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Right Vex Encoder")),
                new PIDConstants(0.0175, 0.001, 0.0008, 0),
                242.11);

        taskPackage = new SimpleTaskPackage("Swerve Tasks",
                frontLeft.swivelTask,
                frontRight.swivelTask,
                backLeft.swivelTask,
                backRight.swivelTask);
        taskPackage.start();
    }

    @Override
    protected void START() throws InterruptedException
    {
        Vector2D desiredRotation;

        while (true)
        {
            desiredRotation = Vector2D.rectangular(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotateBy(-90);

            frontLeft.setVectorTarget(desiredRotation);
            frontRight.setVectorTarget(desiredRotation);
            backLeft.setVectorTarget(desiredRotation);
            backRight.setVectorTarget(desiredRotation);

            Flow.yield();
        }
    }
}
