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
import org.firstinspires.ftc.teamcode.structs.Function;
import org.firstinspires.ftc.teamcode.structs.PIDController;
import org.firstinspires.ftc.teamcode.structs.SwerveModulePIDController;
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
     * A separate method so that their positions can be set ASAP (thus doesn't set off alignment if
     * set manually prior to a match).
     * @param hardware the hardware initializer.
     */
    public static Servo[] getSwerveModuleServos(HardwareInitializer hardware)
    {
        Servo[] moduleServos = new Servo[4];

        moduleServos[0] = hardware.initialize(Servo.class, "Front Left Vex Motor");
        moduleServos[0].setPosition(0.5);

        moduleServos[1] = hardware.initialize(Servo.class, "Back Left Vex Motor");
        moduleServos[1].setPosition(0.5);

        moduleServos[2] = hardware.initialize(Servo.class, "Back Right Vex Motor");
        moduleServos[2].setPosition(0.5);

        moduleServos[3] = hardware.initialize(Servo.class, "Front Right Vex Motor");
        moduleServos[3].setPosition(0.5);

        return moduleServos;
    }

    /**
     * Initializes and passes back the motors, once initialized.  Static so that other classes can
     * access without copy-paste code.
     * @param hardware  The hardware initializer to use for the motors (this is static, after all)
     * @param desiredZeroPowerBehavior  Whether to brake or float on zero power.
     */
    public static EncoderMotor[] getDriveMotors(HardwareInitializer hardware, DcMotor.ZeroPowerBehavior desiredZeroPowerBehavior)
    {
        // The drive motors (to be passed back).
        EncoderMotor[] driveMotors = new EncoderMotor[4];

        // Front left
        driveMotors[0] = new EncoderMotor(
                "Front Left",
                hardware.initialize(DcMotor.class, "Front Left"),
//                new PIDController(.0006, 0, 0, 0, PIDController.TimeUnits.MILLISECONDS, 80, -1, 1),
                new Function()
                {
                    public double value(double input)
                    {
                        return Math.signum(input) * (.00006 * Math.pow(Math.abs(input), 2));
                    }
                },
                50,
                475, 7.62, desiredZeroPowerBehavior);
//        frontLeftDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Back left
        driveMotors[1] = new EncoderMotor(
                "Back Left",
                hardware.initialize(DcMotor.class, "Back Left"),
//                new PIDController(.0006, 0, 0, 0, PIDController.TimeUnits.MILLISECONDS, 80, -1, 1),
                new Function()
                {
                    public double value(double input)
                    {
                        return Math.signum(input) * (.000065 * Math.pow(Math.abs(input), 2));
                    }
                },
                50,
                202, 7.62, desiredZeroPowerBehavior);
//        backLeftDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Back right
        driveMotors[2] = new EncoderMotor(
                "Back Right",
                hardware.initialize(DcMotor.class, "Back Right"),
//                new PIDController(.0006, 0, 0, 0, PIDController.TimeUnits.MILLISECONDS, 80, -1, 1),
                new Function()
                {
                    public double value(double input)
                    {
                        return Math.signum(input) * (.00006 * Math.pow(Math.abs(input), 2));
                    }
                },
                50,
                475, 7.62, desiredZeroPowerBehavior);
//        backRightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Front right
        driveMotors[3] = new EncoderMotor(
                "Front Right",
                hardware.initialize(DcMotor.class, "Front Right"),
//                new PIDController(.0006, 0, 0, 0, PIDController.TimeUnits.MILLISECONDS, 80, -1, 1),
                new Function()
                {
                    public double value(double input)
                    {
                        return Math.signum(input) * (.00006 * Math.pow(Math.abs(input), 2));
                    }
                },
                50,
                202, 7.62, desiredZeroPowerBehavior);
//        frontRightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        return driveMotors;
    }

    /**
     * Provides all swerve modules, linked to the appropriate drive motors.
     * @param hardware The hardware initializer to use.
     * @param zeroPowerBehavior The zero power behavior for the motors.
     * @return
     */
    public static SwerveModule[] getSwerveModules(HardwareInitializer hardware, ControlMode controlMode, DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        Servo[] swerveModuleServos = getSwerveModuleServos(hardware);
        EncoderMotor[] driveMotors = getDriveMotors(hardware, zeroPowerBehavior);

        SwerveModule[] swerveModules = new SwerveModule[4];

        swerveModules[0] = new SwerveModule(
                "Front Left",
                driveMotors[0],
                swerveModuleServos[0],
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Left Vex Encoder")),
                new SwerveModulePIDController(0.0052, 0, 0, .5, SwerveModulePIDController.TimeUnits.MILLISECONDS, 80, -.5, .5, .95),
//                new Function()
//                {
//                    public double value(double input)
//                    {
//                        if (Math.abs(input) < 1)
//                            return 0;
//                        return Math.signum(input) * (.05 + .003 * Math.abs(input));
//                    }
//                },
//                50,
                56);

        swerveModules[1] = new SwerveModule(
                "Back Left",
                driveMotors[1],
                swerveModuleServos[1],
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Left Vex Encoder")),
                new SwerveModulePIDController(0.0062, 0, 0, .5, SwerveModulePIDController.TimeUnits.MILLISECONDS, 80, -.5, .5, .95),
//                new Function()
//                {
//                    public double value(double input)
//                    {
//                        if (Math.abs(input) < 1)
//                            return 0;
//                        return Math.signum(input) * (.05 + .003 * Math.abs(input));
//                    }
//                },
//                50,
                71);

        swerveModules[2] = new SwerveModule(
                "Back Right",
                driveMotors[2],
                swerveModuleServos[2],
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Right Vex Encoder")),
                new SwerveModulePIDController(0.007, 0, 0, .5, SwerveModulePIDController.TimeUnits.MILLISECONDS, 80, -.5, .5, .95),
//                new Function()
//                {
//                    public double value(double input)
//                    {
//                        if (Math.abs(input) < 1)
//                            return 0;
//                        return Math.signum(input) * (.075 + .0035 * Math.abs(input));
//                    }
//                },
//                50,
                132);

        swerveModules[3] = new SwerveModule(
                "Front Right",
                driveMotors[3],
                swerveModuleServos[3],
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Right Vex Encoder")),
                new SwerveModulePIDController(0.0056, 0, 0, .5, SwerveModulePIDController.TimeUnits.MILLISECONDS, 80, -.5, .5, .95),
//                new Function()
//                {
//                    public double value(double input)
//                    {
//                        if (Math.abs(input) < 1)
//                            return 0;
//                        return Math.signum(input) * (.05 + .003 * Math.abs(input));
//                    }
//                },
//                50,
                96);

        return swerveModules;
    }

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

        // Instantiate the swerve drive.
        swerveDrive = new SwerveDrive(this, getSwerveModules(hardware, controlMode, /*controlMode == ControlMode.AUTONOMOUS ? DcMotor.ZeroPowerBehavior.BRAKE : */DcMotor.ZeroPowerBehavior.FLOAT));

        // Init the ADAFRUIT gyro.
//        gyro = new HankuTankuIMU(hardware.map.get(BNO055IMU.class, "IMU"));

        if (controlMode == ControlMode.AUTONOMOUS)
        {
            // Get front and back sensors.
            frontRangeSensor = null; //new SmarterRangeSensor(hardware.initialize(ModernRoboticsI2cRangeSensor.class, "Front Range Sensor"), 0x10);
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
            // Relic Arm init
            relicSystem = null; //new RelicSystem(hardware.initialize(DcMotor.class, "Relic Arm"), hardware.initialize(Servo.class, "Relic Rotator"), hardware.initialize(Servo.class, "Relic Grabber"));
        else
            relicSystem = null;

        // Flipper init
        flipper = new Flipper(hardware.initialize(Servo.class, "Left Flipper"), hardware.initialize(Servo.class, "Right Flipper"), hardware.initialize(Servo.class, "Glyph Holder"));

        // Ball knocker init
        ballKnocker = new BallKnocker(hardware.initialize(Servo.class, "Knocker Holder"), hardware.initialize(Servo.class, "Mini Knocker"));

        if (controlMode == ControlMode.AUTONOMOUS)
            // Lights init
            lights = new LightingSystem(hardware.initialize(DcMotor.class, "Lights"));
        else
            lights = null;
    }
}
