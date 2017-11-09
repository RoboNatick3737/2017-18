package org.firstinspires.ftc.teamcode.programs.finalbot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.MRGyro;
import org.firstinspires.ftc.teamcode.hardware.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.SwerveDrive;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.SwerveWheel;

import hankextensions.Core;
import hankextensions.phonesensors.AndroidGyro;

import org.firstinspires.ftc.teamcode.hardware.EncoderMotor;

public abstract class HardwareBase extends Core
{
    protected SwerveDrive swerveDrive;

    @Override
    protected void HARDWARE() throws InterruptedException
    {
        // Init the android gyro (make sure to call start()).
        AndroidGyro androidGyro = new AndroidGyro();
        androidGyro.start();
        androidGyro.zero();

        // MRGyro mrGyro = new MRGyro(initHardwareDevice(GyroSensor.class, "MR Gyroscope"));

        // All of the drive motors and their respective PID.
        EncoderMotor frontLeftDrive = new EncoderMotor(
                "Front Left",
                initHardwareDevice(DcMotor.class, "Front Left"),
                new PIDConstants(.0008, 0, 0, 0),
                407, 7.62);

        EncoderMotor frontRightDrive = new EncoderMotor(
                "Front Right",
                initHardwareDevice(DcMotor.class, "Front Right"),
                new PIDConstants(.0008, 0, 0, 0),
                202, 7.62);

        EncoderMotor backLeftDrive = new EncoderMotor(
                "Back Left",
                initHardwareDevice(DcMotor.class, "Back Left"),
                new PIDConstants(.0008, 0, 0, 0),
                202, 7.62);

        EncoderMotor backRightDrive = new EncoderMotor(
                "Back Right",
                initHardwareDevice(DcMotor.class, "Back Right"),
                new PIDConstants(.0008, 0, 0, 0),
                475, 7.62);


        // All of the SwerveWheels (which align on independent threads)
        SwerveWheel frontLeft = new SwerveWheel(
                "Front Left",
                frontLeftDrive,
                initHardwareDevice(Servo.class, "Front Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Left Vex Encoder")),
                new PIDConstants(0.011042, 0, 0.000508, 5.194),
                61.58);

        SwerveWheel frontRight = new SwerveWheel(
                "Front Right",
                frontRightDrive,
                initHardwareDevice(Servo.class, "Front Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Right Vex Encoder")),
                new PIDConstants(0.010465, 0, 0.000645, 2.5),
                233.66);

        SwerveWheel backLeft = new SwerveWheel(
                "Back Left",
                backLeftDrive,
                initHardwareDevice(Servo.class, "Back Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Left Vex Encoder")),
                new PIDConstants(0.0107, 0, 0.000604, 2.85),
                47.5);

        SwerveWheel backRight = new SwerveWheel(
                "Back Right",
                backRightDrive,
                initHardwareDevice(Servo.class, "Back Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Right Vex Encoder")),
                new PIDConstants(0.0087, 0, 0.000269, 5.678),
                257.9);

        // Creates the swerve drive with the correct joystick.
        swerveDrive = new SwerveDrive(androidGyro, frontLeft, frontRight, backLeft, backRight);
    }
}
