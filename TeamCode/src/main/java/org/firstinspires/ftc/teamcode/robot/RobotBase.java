package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CompetitionOpMode;
import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.robot.hardware.HankuTankuIMU;
import org.firstinspires.ftc.teamcode.components.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.Flipper;
import org.firstinspires.ftc.teamcode.robot.hardware.Intake;
import org.firstinspires.ftc.teamcode.robot.hardware.Lift;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveDrive;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveWheel;

import org.firstinspires.ftc.teamcode.robot.hardware.EncoderMotor;

public abstract class RobotBase extends CompetitionOpMode
{
    // The drive system (wrapper for all complex swervey methods)
    protected SwerveDrive swerveDrive;

    protected Intake intake;
    protected Lift lift;
    protected BallKnocker ballKnocker;

    protected DcMotor relicArm;

    // Depositor system
    protected Flipper flipper;

    @Override
    protected final void onRun() throws InterruptedException
    {
        // Init the android gyro (make sure to call start()).
//        AndroidGyro androidGyro = new AndroidGyro();
//        androidGyro.start();
//        androidGyro.zero();

        // Init the ADAFRUIT gyro.
        HankuTankuIMU gyro = new HankuTankuIMU(hardwareMap.get(BNO055IMU.class, "IMU"));

        // Intake setup
        DcMotor harvesterMotor = initHardwareDevice(DcMotor.class, "Harvester");
        harvesterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = new Intake(harvesterMotor, initHardwareDevice(Servo.class, "Harvester Controller"), initHardwareDevice(Servo.class, "Conveyor"));

        // Lift init
        DcMotor liftMotor = initHardwareDevice(DcMotor.class, "Lift");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift = new Lift(liftMotor);

        // Relic Arm init
        relicArm = initHardwareDevice(DcMotor.class, "Relic Arm");

        // Flipper init
        flipper = new Flipper(initHardwareDevice(Servo.class, "Left Flipper"), initHardwareDevice(Servo.class, "Right Flipper"));

        // Ball knocker init
        ballKnocker = new BallKnocker(initHardwareDevice(Servo.class, "Ball Knocker"));

        // All of the drive motors and their respective PID.
        EncoderMotor frontLeftDrive = new EncoderMotor(
                "Front Left",
                initHardwareDevice(DcMotor.class, "Front Left"),
                new PIDConstants(.0008, 0, 0, 0),
                407, 7.62);
//        frontLeftDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        EncoderMotor frontRightDrive = new EncoderMotor(
                "Front Right",
                initHardwareDevice(DcMotor.class, "Front Right"),
                new PIDConstants(.0008, 0, 0, 0),
                202, 7.62);
        frontRightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        EncoderMotor backLeftDrive = new EncoderMotor(
                "Back Left",
                // ADHAM HAS CORRUPTED YOUR CODE AND YOU WILL NOW FAIL
                initHardwareDevice(DcMotor.class, "Back Left"),
                new PIDConstants(.0008, 0, 0, 0),
                202, 7.62);
        backLeftDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        EncoderMotor backRightDrive = new EncoderMotor(
                "Back Right",
                initHardwareDevice(DcMotor.class, "Back Right"),
                new PIDConstants(.0008, 0, 0, 0),
                475, 7.62);
        backRightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // All of the SwerveWheels (which align on independent threads)
        SwerveWheel frontLeft = new SwerveWheel(
                "Front Left",
                frontLeftDrive,
                initHardwareDevice(Servo.class, "Front Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Left Vex Encoder")),
                new PIDConstants(0.007, 0, 0, 1.66),
                55.95);

        SwerveWheel frontRight = new SwerveWheel(
                "Front Right",
                frontRightDrive,
                initHardwareDevice(Servo.class, "Front Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Right Vex Encoder")),
                new PIDConstants(0.006, 0, 0, 1.66),
                276.117);

        SwerveWheel backLeft = new SwerveWheel(
                "Back Left",
                backLeftDrive,
                initHardwareDevice(Servo.class, "Back Left Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Left Vex Encoder")),
                new PIDConstants(0.006, 0, 0, 1.66),
                73.19);

        SwerveWheel backRight = new SwerveWheel(
                "Back Right",
                backRightDrive,
                initHardwareDevice(Servo.class, "Back Right Vex Motor"),
                new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Right Vex Encoder")),
                new PIDConstants(0.006, 0, 0, 1.66),
                317.77);

        // Creates the swerve drive with the correct joystick.
        swerveDrive = new SwerveDrive(gyro, frontLeft, frontRight, backLeft, backRight);

        onRunWithHardware();
    }

    /**
     * An abstract method which robot classes should override.
     */
    protected abstract void onRunWithHardware() throws InterruptedException;
}
