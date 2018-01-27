package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.robot.hardware.LightingSystem;
import org.firstinspires.ftc.teamcode.robot.hardware.RelicSystem;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveModule;
import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.Flipper;
import org.firstinspires.ftc.teamcode.robot.hardware.Intake;
import org.firstinspires.ftc.teamcode.robot.hardware.Lift;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveDrive;

import org.firstinspires.ftc.teamcode.robot.hardware.EncoderMotor;

import hankextensions.hardware.HardwareInitializer;
import hankextensions.hardware.SmarterRangeSensor;
import hankextensions.phonesensors.AndroidGyro;
import hankextensions.phonesensors.Gyro;

/**
 * Wrapper class for all robot hardware.
 */
public class Robot
{
    public enum ControlMode {
        TELEOP, // no point in including sensors
        AUTONOMOUS // no point in including relic arm
    }
    public final ControlMode controlMode;

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

    // Autonomous sensors
    public final SmarterRangeSensor frontRangeSensor, backRangeSensor;

    /**
     * Initializes the whole robot.
     */
    public Robot(HardwareInitializer hardware, ControlMode controlMode) throws InterruptedException
    {
        // Swerve drive needs to know this as well.
        this.controlMode = controlMode;

        // Init the android gyro (make sure to call start()).
        AndroidGyro androidGyro = new AndroidGyro();
        androidGyro.start();
        androidGyro.initAntiDrift();
        gyro = androidGyro;

        // Init the ADAFRUIT gyro.
//        gyro = new HankuTankuIMU(hardware.map.get(BNO055IMU.class, "IMU"));

        if (controlMode == ControlMode.AUTONOMOUS)
        {
            // Get front and back sensors.
            frontRangeSensor = new SmarterRangeSensor(hardware.initialize(ModernRoboticsI2cRangeSensor.class, "Front Range Sensor"), 0x10);
            backRangeSensor = new SmarterRangeSensor(hardware.initialize(ModernRoboticsI2cRangeSensor.class, "Back Range Sensor"), 0x2c);
        }
        else
        {
            frontRangeSensor = null;
            backRangeSensor = null;
        }

        // Intake setup
        intake = new Intake(hardware.initialize(Servo.class, "Left Harvester"), hardware.initialize(Servo.class, "Right Harvester"), hardware.initialize(DcMotor.class, "Secondary Harvester"));

        // Lift init
        DcMotor liftMotor = hardware.initialize(DcMotor.class, "Lift");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = new Lift(liftMotor);

        if (controlMode == ControlMode.TELEOP)
        {
            // Relic Arm init
            relicSystem = null; //new RelicSystem(hardware.initialize(DcMotor.class, "Relic Arm"), hardware.initialize(Servo.class, "Relic Rotator"), hardware.initialize(Servo.class, "Relic Grabber"));
        }
        else
        {
            relicSystem = null;
        }

        // Flipper init
        flipper = new Flipper(hardware.initialize(Servo.class, "Left Flipper"), hardware.initialize(Servo.class, "Right Flipper"), hardware.initialize(Servo.class, "Glyph Holder"));

        // Ball knocker init
        ballKnocker = new BallKnocker(hardware.initialize(Servo.class, "Knocker Holder"), hardware.initialize(Servo.class, "Mini Knocker"));

        if (controlMode == ControlMode.AUTONOMOUS)
            // Lights init
            lights = new LightingSystem(hardware.initialize(DcMotor.class, "Lights"));
        else
            lights = null;


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
        SwerveModule frontLeft = new SwerveModule(
                "Front Left",
                frontLeftDrive,
                hardware.initialize(Servo.class, "Front Left Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Left Vex Encoder")),
                new PIDConstants(0.007, 0, 0, .5, 40000000),
                57);

        SwerveModule frontRight = new SwerveModule(
                "Front Right",
                frontRightDrive,
                hardware.initialize(Servo.class, "Front Right Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Right Vex Encoder")),
                new PIDConstants(0.08, 0, 0, .5, 40000000),
                97);

        SwerveModule backLeft = new SwerveModule(
                "Back Left",
                backLeftDrive,
                hardware.initialize(Servo.class, "Back Left Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Left Vex Encoder")),
                new PIDConstants(0.007, 0, 0, .5, 40000000),
                73);

        SwerveModule backRight = new SwerveModule(
                "Back Right",
                backRightDrive,
                hardware.initialize(Servo.class, "Back Right Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Right Vex Encoder")),
                new PIDConstants(0.007, 0, 0, .5, 40000000),
                137.78);

        // Creates the swerve drive with the correct joystick.
        swerveDrive = new SwerveDrive(this, frontLeft, frontRight, backLeft, backRight);
    }
}
