package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.SwerveDrive;
import org.firstinspires.ftc.teamcode.components.SwerveWheel;
import org.firstinspires.ftc.teamcode.programs.Core;
import org.firstinspires.ftc.teamcode.sdkextensions.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.sdkextensions.hardware.Gyro;

public class TestSwerveDrive extends Core
{
    private SwerveDrive swerveDrive;

    @Override
    protected void INITIALIZE() throws InterruptedException
    {
        Gyro gyro = new Gyro(initHardwareDevice(GyroSensor.class, "Gyroscope"));
        SwerveWheel frontLeft = new SwerveWheel(
                "Front Left",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Left")),
                initHardwareDevice(Servo.class, "Front Left Vex"),
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Left Vex Encoder")));
        SwerveWheel frontRight = new SwerveWheel(
                "Front Right",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Right")),
                initHardwareDevice(Servo.class, "Front Right Vex"),
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Front Right Vex Encoder")));
        SwerveWheel backLeft = new SwerveWheel(
                "Back Left",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Left")),
                initHardwareDevice(Servo.class, "Back Left Vex"),
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Left Vex Encoder")));
        SwerveWheel backRight = new SwerveWheel(
                "Back Right",
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Right")),
                initHardwareDevice(Servo.class, "Back Right Vex"),
                new EncoderMotor(initHardwareDevice(DcMotor.class, "Back Right Vex Encoder")));

        // Creates the swerve drive with the correct joystick.
        swerveDrive = new SwerveDrive(gyro, frontLeft, frontRight, backLeft, backRight, gamepad1);
    }

    @Override
    protected void START() throws InterruptedException
    {

    }
}
