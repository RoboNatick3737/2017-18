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

        // All of the SwerveWheels (which align on independent threads)
        SwerveWheel frontLeft = new SwerveWheel(
                "Front Left",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Left")),
                initHardwareDevice(Servo.class, "Front Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Left Vex Encoder")),
                new PIDConstants(0.015042, 0, 0.000608, 5.194),
                61.58);

        SwerveWheel frontRight = new SwerveWheel(
                "Front Right",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Right")),
                initHardwareDevice(Servo.class, "Front Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Right Vex Encoder")),
                new PIDConstants(0.012465, 0, 0.000945, 2.5),
                228.38);

        SwerveWheel backLeft = new SwerveWheel(
                "Back Left",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Left")),
                initHardwareDevice(Servo.class, "Back Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Left Vex Encoder")),
                new PIDConstants(0.0127, 0, 0.000704, 2.85),
                43.636);

        SwerveWheel backRight = new SwerveWheel(
                "Back Right",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Right")),
                initHardwareDevice(Servo.class, "Back Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Right Vex Encoder")),
                new PIDConstants(0.01404, 0, 0.000669, 5.678),
                257.24);

        // Creates the swerve drive with the correct joystick.
        swerveDrive = new SwerveDrive(androidGyro, frontLeft, frontRight, backLeft, backRight);
    }
}
