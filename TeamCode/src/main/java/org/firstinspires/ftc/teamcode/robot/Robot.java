package org.firstinspires.ftc.teamcode.robot;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.threading.TimeMeasure;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.robot.hardware.LightingSystem;
import org.firstinspires.ftc.teamcode.robot.hardware.RelicSystem;
import org.firstinspires.ftc.teamcode.robot.hardware.SwomniModule;
import org.firstinspires.ftc.teamcode.robot.hardware.SwomniDrive;
import hankutanku.math.Function;
import hankutanku.math.LimitedUpdateRateFunction;
import hankutanku.math.ModifiedPIDController;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.Flipper;
import org.firstinspires.ftc.teamcode.robot.hardware.Intake;
import org.firstinspires.ftc.teamcode.robot.hardware.Lift;

import org.firstinspires.ftc.teamcode.robot.hardware.EncoderMotor;

import hankutanku.EnhancedOpMode;
import hankutanku.hardware.HardwareInitializer;
import hankutanku.hardware.SmarterRangeSensor;
import hankutanku.phonesensors.AndroidGyro;
import hankutanku.phonesensors.Gyro;
import hankutanku.math.Angle;

/**
 * Wrapper class for all robot hardware.
 */
public class Robot
{
    // The drive system (wrapper for all complex swervey methods)
    public final SwomniDrive swomniDrive;

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
                new LimitedUpdateRateFunction<Double>()
                {
                    @Override
                    public TimeMeasure getUpdateRate()
                    {
                        return new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 50);
                    }

                    public Double value(double input)
                    {
                        return Math.signum(input) * (6e-5 * Math.pow(Math.abs(input), 2));
                    }
                },
                475, 7.62, desiredZeroPowerBehavior);
//        frontLeftDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Back left
        driveMotors[1] = new EncoderMotor(
                "Back Left",
                hardware.initialize(DcMotor.class, "Back Left"),
//                new PIDController(.0006, 0, 0, 0, PIDController.TimeUnits.MILLISECONDS, 80, -1, 1),
                new LimitedUpdateRateFunction<Double>()
                {
                    @Override
                    public TimeMeasure getUpdateRate()
                    {
                        return new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 50);
                    }

                    public Double value(double input)
                    {
                        return Math.signum(input) * (6.5e-5 * Math.pow(Math.abs(input), 2));
                    }
                },
                202, 7.62, desiredZeroPowerBehavior);
//        backLeftDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Back right
        driveMotors[2] = new EncoderMotor(
                "Back Right",
                hardware.initialize(DcMotor.class, "Back Right"),
//                new PIDController(.0006, 0, 0, 0, PIDController.TimeUnits.MILLISECONDS, 80, -1, 1),
                new LimitedUpdateRateFunction<Double>()
                {
                    @Override
                    public TimeMeasure getUpdateRate() {
                        return new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 50);
                    }

                    public Double value(double input)
                    {
                        return Math.signum(input) * (6e-5 * Math.pow(Math.abs(input), 2));
                    }
                },
                475, 7.62, desiredZeroPowerBehavior);
//        backRightDrive.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Front right
        driveMotors[3] = new EncoderMotor(
                "Front Right",
                hardware.initialize(DcMotor.class, "Front Right"),
//                new PIDController(.0006, 0, 0, 0, PIDController.TimeUnits.MILLISECONDS, 80, -1, 1),
                new LimitedUpdateRateFunction<Double>()
                {
                    @Override
                    public TimeMeasure getUpdateRate() {
                        return new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 50);
                    }

                    public Double value(double input)
                    {
                        return Math.signum(input) * (6e-5 * Math.pow(Math.abs(input), 2));
                    }
                },
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
    public static SwomniModule[] getSwerveModules(HardwareInitializer hardware, EnhancedOpMode.AutoOrTeleop opModeSituation, DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        Servo[] swerveModuleServos = getSwerveModuleServos(hardware);
        EncoderMotor[] driveMotors = getDriveMotors(hardware, zeroPowerBehavior);

        SwomniModule[] swomniModules = new SwomniModule[4];

        swomniModules[0] = new SwomniModule(
                "Front Left",
                driveMotors[0],
                swerveModuleServos[0],
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Left Vex Encoder")),

                // For swerve drive mode.
                new ModifiedPIDController(0.0052, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                // For holonomic mode
                new ModifiedPIDController(0.0062, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                // For tank drive mode
                new ModifiedPIDController(0.0062, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                Angle.degrees(56),
                5e-4);

        swomniModules[1] = new SwomniModule(
                "Back Left",
                driveMotors[1],
                swerveModuleServos[1],
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Left Vex Encoder")),

                // For swerve drive mode.
                new ModifiedPIDController(0.0062, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                // For holonomic mode
                new ModifiedPIDController(0.0062, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                // For tank drive mode
                new ModifiedPIDController(0.0062, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                Angle.degrees(71),
                1e-3);

        swomniModules[2] = new SwomniModule(
                "Back Right",
                driveMotors[2],
                swerveModuleServos[2],
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Right Vex Encoder")),

                // For swerve drive mode.
                new ModifiedPIDController(0.007, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                // For holonomic mode
                new ModifiedPIDController(0.007, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                // For tank drive mode
                new ModifiedPIDController(0.007, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                Angle.degrees(132),
                1e-3);

        swomniModules[3] = new SwomniModule(
                "Front Right",
                driveMotors[3],
                swerveModuleServos[3],
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Right Vex Encoder")),

                // For swerve drive mode.
                new ModifiedPIDController(0.0056, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                // For holonomic mode
                new ModifiedPIDController(0.0056, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                // For tank drive mode
                new ModifiedPIDController(0.0056, 0, 0, .5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 80), -.5, .5, .95),

                Angle.degrees(96),
                5e-4);

        return swomniModules;
    }

    /**
     * Initializes the whole robot.
     */
    public Robot(HardwareInitializer hardware, EnhancedOpMode.AutoOrTeleop opModeSituation) throws InterruptedException
    {
        // Init the android gyro (make sure to call start()).
        AndroidGyro androidGyro = new AndroidGyro();
        androidGyro.start();
        androidGyro.initAntiDrift();
        gyro = androidGyro;

        // Init the ADAFRUIT gyro.
//        gyro = new HankuTankuIMU(hardware.map.get(BNO055IMU.class, "IMU"));

        // Instantiate the swerve drive.
        swomniDrive = new SwomniDrive(opModeSituation, gyro, getSwerveModules(hardware, opModeSituation, DcMotor.ZeroPowerBehavior.FLOAT));

        if (opModeSituation == EnhancedOpMode.AutoOrTeleop.AUTONOMOUS)
        {
            // Get front and back sensors.
            frontRangeSensor = null; //new SmarterRangeSensor(hardware.initialize(ModernRoboticsI2cRangeSensor.class, "Front Range Sensor"), 0x10);
            backRangeSensor = new SmarterRangeSensor(hardware.initialize(ModernRoboticsI2cRangeSensor.class, "Back Range Sensor"), 0x2c);

            if (backRangeSensor.getForwardDist() >= 255)
                LoggingBase.instance.lines("RANGE SENSOR FUCKED UP REEEEE (power cycle the robot)");
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

        if (opModeSituation == EnhancedOpMode.AutoOrTeleop.TELEOP)
            // Relic Arm init
            relicSystem = new RelicSystem(hardware.initialize(DcMotor.class, "Relic Extender"), hardware.initialize(Servo.class, "Relic Grabber"));
        else
            relicSystem = null;

        // Flipper init
        flipper = new Flipper(hardware.initialize(Servo.class, "Left Flipper"), hardware.initialize(Servo.class, "Right Flipper"), hardware.initialize(Servo.class, "Glyph Clamp"));

        // Ball knocker init
        ballKnocker = new BallKnocker(hardware.initialize(Servo.class, "Knocker Holder"), hardware.initialize(Servo.class, "Mini Knocker"));

        // Lights init
        lights = new LightingSystem(hardware.initialize(DcMotor.class, "Lights"));
    }
}
