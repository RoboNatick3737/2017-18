package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.robot.hardware.LightingSystem;
import org.firstinspires.ftc.teamcode.robot.hardware.RelicSystem;
import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.Flipper;
import org.firstinspires.ftc.teamcode.robot.hardware.Intake;
import org.firstinspires.ftc.teamcode.robot.hardware.Lift;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveDrive;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveWheel;

import org.firstinspires.ftc.teamcode.robot.hardware.EncoderMotor;

import hankextensions.hardware.HardwareInitializer;
import hankextensions.phonesensors.AndroidGyro;
import hankextensions.phonesensors.Gyro;

/**
 * Wrapper class for all robot hardware.
 */
public class Robot
{
    // The drive system (wrapper for all complex swervey methods)
    public final SwerveDrive swerveDrive;

    // All other robot components (modular)
    public Gyro gyro; // Set by one class for straight drive testing.
    public final Intake intake;
    public final Flipper flipper;
    public final Lift lift;
    public final BallKnocker ballKnocker;
    public final RelicSystem relicSystem;
    public final LightingSystem lights;

    /**
     * Initializes the whole robot.
     */
    public Robot(HardwareInitializer hardware) throws InterruptedException
    {
        // Init the android gyro (make sure to call start()).
        AndroidGyro androidGyro = new AndroidGyro();
        androidGyro.start();
        androidGyro.initAntiDrift();
        gyro = androidGyro;

        // Init the ADAFRUIT gyro.
//        gyro = new HankuTankuIMU(hardware.map.get(BNO055IMU.class, "IMU"));

        // Intake setup
        DcMotor harvesterMotor = hardware.initialize(DcMotor.class, "Harvester");
        harvesterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = new Intake(harvesterMotor, hardware.initialize(Servo.class, "Conveyor"));

        // Lift init
        DcMotor liftMotor = hardware.initialize(DcMotor.class, "Lift");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = new Lift(liftMotor);

        // Relic Arm init
        relicSystem = new RelicSystem(hardware.initialize(DcMotor.class, "Relic Arm"), hardware.initialize(Servo.class, "Relic Rotator"), hardware.initialize(Servo.class, "Relic Grabber"));

        // Flipper init
        flipper = new Flipper(hardware.initialize(Servo.class, "Left Flipper"), hardware.initialize(Servo.class, "Right Flipper"), hardware.initialize(Servo.class, "Glyph Holder"));

        // Ball knocker init
        ballKnocker = null; //new BallKnocker(hardware.initialize(Servo.class, "Ball Knocker"));

        // Lights
        lights = new LightingSystem(hardware.initialize(DcMotor.class, "Lights"));


        // All of the drive motors and their respective PID.
        EncoderMotor frontLeftDrive = new EncoderMotor(
                "Front Left",
                hardware.initialize(DcMotor.class, "Front Left"),
                new PIDConstants(.0006, 0, 0, 0, 40000000),
                475, 7.62);
//        frontLeftDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        EncoderMotor frontRightDrive = new EncoderMotor(
                "Front Right",
                hardware.initialize(DcMotor.class, "Front Right"),
                new PIDConstants(.0006, 0, 0, 0, 40000000),
                202, 7.62);
//        frontRightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        EncoderMotor backLeftDrive = new EncoderMotor(
                "Back Left",
                hardware.initialize(DcMotor.class, "Back Left"),
                new PIDConstants(.0006, 0, 0, 0, 40000000),
                202, 7.62);
//        backLeftDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        EncoderMotor backRightDrive = new EncoderMotor(
                "Back Right",
                hardware.initialize(DcMotor.class, "Back Right"),
                new PIDConstants(.0006, 0, 0, 0, 40000000),
                475, 7.62);
//        backRightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // All of the SwerveWheels (which align on independent threads)
        SwerveWheel frontLeft = new SwerveWheel(
                "Front Left",
                frontLeftDrive,
                hardware.initialize(Servo.class, "Front Left Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Left Vex Encoder")),
                new PIDConstants(0.009, 0, 0, .5, 40000000),
                57);

        SwerveWheel frontRight = new SwerveWheel(
                "Front Right",
                frontRightDrive,
                hardware.initialize(Servo.class, "Front Right Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Right Vex Encoder")),
                new PIDConstants(0.01, 0, 0, .5, 40000000),
                97);

        SwerveWheel backLeft = new SwerveWheel(
                "Back Left",
                backLeftDrive,
                hardware.initialize(Servo.class, "Back Left Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Left Vex Encoder")),
                new PIDConstants(0.009, 0, 0, .5, 40000000),
                73);

        SwerveWheel backRight = new SwerveWheel(
                "Back Right",
                backRightDrive,
                hardware.initialize(Servo.class, "Back Right Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Right Vex Encoder")),
                new PIDConstants(0.009, 0, 0, .5, 40000000),
                137.78);

        // Creates the swerve drive with the correct joystick.
        swerveDrive = new SwerveDrive(gyro, frontLeft, frontRight, backLeft, backRight);
    }
}
