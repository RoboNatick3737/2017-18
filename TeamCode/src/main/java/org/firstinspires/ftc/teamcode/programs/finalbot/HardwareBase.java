package org.firstinspires.ftc.teamcode.programs.finalbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.SwerveDrive;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.SwerveWheel;

import hankextensions.Core;
import hankextensions.phonesensors.AndroidGyro;

import org.firstinspires.ftc.teamcode.hardware.EncoderMotor;

public abstract class HardwareBase extends Core
{
    private SwerveDrive swerveDrive;

    @Override
    protected void HARDWARE() throws InterruptedException
    {
        // Init the android gyro (make sure to call start()).
        AndroidGyro androidGyro = new AndroidGyro();
        androidGyro.start();

        // All of the SwerveWheels (which align on independent threads)
        SwerveWheel frontLeft = new SwerveWheel(
                "Front Left",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Left")),
                initHardwareDevice(Servo.class, "Front Left Vex"),
                initHardwareDevice(AbsoluteEncoder.class, "Front Left Vex Encoder"));

        SwerveWheel frontRight = new SwerveWheel(
                "Front Right",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Right")),
                initHardwareDevice(Servo.class, "Front Right Vex"),
                initHardwareDevice(AbsoluteEncoder.class, "Front Right Vex Encoder"));

        SwerveWheel backLeft = new SwerveWheel(
                "Back Left",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Left")),
                initHardwareDevice(Servo.class, "Back Left Vex"),
                initHardwareDevice(AbsoluteEncoder.class, "Back Left Vex Encoder"));

        SwerveWheel backRight = new SwerveWheel(
                "Back Right",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Right")),
                initHardwareDevice(Servo.class, "Back Right Vex"),
                initHardwareDevice(AbsoluteEncoder.class, "Back Right Vex Encoder"));

        // Creates the swerve drive with the correct joystick.
        swerveDrive = new SwerveDrive(androidGyro, frontLeft, frontRight, backLeft, backRight, gamepad1);
    }
}
