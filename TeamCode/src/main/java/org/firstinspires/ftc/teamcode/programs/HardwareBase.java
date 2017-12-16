package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.HankuTankuIMU;
import org.firstinspires.ftc.teamcode.hardware.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.programs.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.programs.hardware.SwerveDrive;
import org.firstinspires.ftc.teamcode.programs.hardware.SwerveWheel;

import hankextensions.RobotCore;
import hankextensions.phonesensors.AndroidGyro;

import org.firstinspires.ftc.teamcode.hardware.EncoderMotor;

public abstract class HardwareBase extends RobotCore
{
    // The drive system (wrapper for all complex swervey methods)
    protected SwerveDrive swerveDrive;

    // Harvester and lift
    protected DcMotor harvester;
    protected DcMotor lift;

    // Relic system
    protected DcMotor relicArm;

    // Depositor system
    protected Servo leftFlipper, rightFlipper;
    protected Servo conveyor;

    @Override
    protected void HARDWARE() throws InterruptedException
    {
        // Init the android gyro (make sure to call start()).
//        AndroidGyro androidGyro = new AndroidGyro();
//        androidGyro.start();
//        androidGyro.zero();

        // Init the ADAFRUIT gyro.
        HankuTankuIMU gyro = new HankuTankuIMU(hardwareMap.get(BNO055IMU.class, "IMU"));

        // Harvester and Lift
        harvester = initHardwareDevice(DcMotor.class, "Harvester");
        lift = initHardwareDevice(DcMotor.class, "Lift");

        // Relic Arm
        relicArm = initHardwareDevice(DcMotor.class, "Relic Arm");

        // Flipper and conveyor.
        leftFlipper = initHardwareDevice(Servo.class, "Left Flipper");
        rightFlipper = initHardwareDevice(Servo.class, "Right Flipper");
        conveyor = initHardwareDevice(Servo.class, "Conveyor");

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
                new PIDConstants(0.007, 0, 0, 1.66),
                32.55);

        SwerveWheel frontRight = new SwerveWheel(
                "Front Right",
                frontRightDrive,
                initHardwareDevice(Servo.class, "Front Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Right Vex Encoder")),
                new PIDConstants(0.006, 0, 0, 1.66),
                192.3);

        SwerveWheel backLeft = new SwerveWheel(
                "Back Left",
                backLeftDrive,
                initHardwareDevice(Servo.class, "Back Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Left Vex Encoder")),
                new PIDConstants(0.006, 0, 0, 1.66),
                51);

        SwerveWheel backRight = new SwerveWheel(
                "Back Right",
                backRightDrive,
                initHardwareDevice(Servo.class, "Back Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Right Vex Encoder")),
                new PIDConstants(0.005, 0, 0, 1.66),
                309.50);

        // Creates the swerve drive with the correct joystick.
        swerveDrive = new SwerveDrive(gyro, frontLeft, frontRight, backLeft, backRight);
    }
}
