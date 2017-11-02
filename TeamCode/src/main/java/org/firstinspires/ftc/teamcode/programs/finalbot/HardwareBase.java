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
                new PIDConstants(0.00001, 0, 0, 0),
                59.44);

        SwerveWheel frontRight = new SwerveWheel(
                "Front Right",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Right")),
                initHardwareDevice(Servo.class, "Front Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Right Vex Encoder")),
                new PIDConstants(0.00001, 0, 0, 0),
                42.22);

        SwerveWheel backLeft = new SwerveWheel(
                "Back Left",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Left")),
                initHardwareDevice(Servo.class, "Back Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Left Vex Encoder")),
                new PIDConstants(0.00001, 0, 0, 0),
                40.47);

        SwerveWheel backRight = new SwerveWheel(
                "Back Right",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Right")),
                initHardwareDevice(Servo.class, "Back Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Right Vex Encoder")),
                new PIDConstants(0.00001, 0, 0, 0),
                242.11);

        // Creates the swerve drive with the correct joystick.
        swerveDrive = new SwerveDrive(androidGyro, frontLeft, frontRight, backLeft, backRight);
    }
}
