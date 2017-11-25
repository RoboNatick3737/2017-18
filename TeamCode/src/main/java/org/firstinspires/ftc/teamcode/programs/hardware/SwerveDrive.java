package org.firstinspires.ftc.teamcode.programs.hardware;

import android.support.annotation.NonNull;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.SimpleTask;
import com.makiah.makiahsandroidlib.threading.SimpleTaskPackage;
import com.qualcomm.robotcore.hardware.Gamepad;

import hankextensions.RobotCore;
import hankextensions.input.GamepadInterface;
import hankextensions.logging.Log;
import hankextensions.phonesensors.Gyro;

import org.firstinspires.ftc.teamcode.hardware.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.hardware.pid.PIDController;
import hankextensions.structs.Vector2D;

public class SwerveDrive
{
    // Physical robot constants.
    private static final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH)); // Will be 45 degrees with perfect square dimensions.
    private static final double[] WHEEL_ORIENTATIONS = {ROBOT_PHI - 90, (180 - ROBOT_PHI) - 90, (180 + ROBOT_PHI) - 90, (360 - ROBOT_PHI) - 90};
    private static final PIDConstants TURN_PID_CONSTANTS = new PIDConstants(.005, 0, 0, 12);

    // Instance specific components.
    private final SwerveWheel frontLeft, frontRight, backLeft, backRight;
    public final Gyro gyro; // Public because teleop can manually reset.
    private final PIDController pidController;

    // Constantly shifting in autonomous and teleop.
    private Vector2D desiredMovement = Vector2D.ZERO;
    private double desiredHeading = 0;

    /**
     * This task package runs on a different thread, continually updating the vectors for each swerve wheel, while also
     * running the tasks which update the orientation of the swerve wheel.
     *
     * They're all in the same package in order to avoid using a bunch of tasks on solely this.
     */
    private final SimpleTaskPackage drivingTasks;
    private final ProcessConsole swerveConsole;

    // The gamepad which controls the bot.
    private GamepadInterface gamepad;

    // The swerve drive constructor, starts all swerve wheel alignment threads.
    public SwerveDrive(Gyro gyro, SwerveWheel frontLeft, SwerveWheel frontRight, SwerveWheel backLeft, SwerveWheel backRight) throws InterruptedException
    {
        this.gyro = gyro;

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        // Construct the task package which will update all of the turning for the swerve motors.
        drivingTasks = new SimpleTaskPackage(RobotCore.instance, "Swerve Turn Alignments");
        drivingTasks.add(this.frontLeft.swivelTask);
        drivingTasks.add(this.frontRight.swivelTask);
        drivingTasks.add(this.backLeft.swivelTask);
        drivingTasks.add(this.backRight.swivelTask);
        drivingTasks.add(new SwerveDriveTask());

        swerveConsole = Log.instance.newProcessConsole("Swerve Console");

        pidController = new PIDController(TURN_PID_CONSTANTS);

        Log.instance.lines("Wheel orientations: " + WHEEL_ORIENTATIONS[0] + ", " + WHEEL_ORIENTATIONS[1] + ", " + WHEEL_ORIENTATIONS[2] + ", " + WHEEL_ORIENTATIONS[3]);

        drivingTasks.run();
    }

    /**
     * Tells the swerve drive how to accomplish teleop.
     * @param gamepad the gamepad which provides instructions.
     */
    public void provideGamepad(Gamepad gamepad)
    {
        this.gamepad = new GamepadInterface(gamepad);
    }

    /**
     * Sets a new desired orientation.
     * @param newlyDesiredHeading the new orientation to align to.
     */
    public void setDesiredHeading(double newlyDesiredHeading)
    {
        this.desiredHeading = newlyDesiredHeading;
    }

    /**
     * Sets new desired translation.
     * @param newlyDesiredMovement the new desired movement vector.
     *
     */
    public void setDesiredMovement(@NonNull Vector2D newlyDesiredMovement)
    {
        this.desiredMovement = newlyDesiredMovement;
    }

    /**
     * Only takes effect if the gamepad has been supplied to this class.
     */
    private void updateTeleopInstructions() throws InterruptedException
    {
        // Rotate by -90 in order to make forward facing zero.
        Vector2D joystickDesiredRotation = gamepad.leftJoystick();
        Vector2D joystickDesiredMovement = gamepad.rightJoystick();

        // Use the left joystick for rotation unless nothing is supplied, in which case check the DPAD.
        if (joystickDesiredRotation.magnitude > .0005)
            setDesiredHeading(joystickDesiredRotation.angle);
        else
        {
            Vector2D dpadDesiredRotation = gamepad.dpad();

            if (dpadDesiredRotation.magnitude > .0005)
                setDesiredHeading(dpadDesiredRotation.angle);
        }

        if (joystickDesiredMovement.magnitude > .0005)
            setDesiredMovement(joystickDesiredMovement);
        else
            setDesiredMovement(Vector2D.ZERO);

        if (gamepad.gamepad.a)
        {
            gyro.calibrate();
            setDesiredHeading(0);
        }

        if (gamepad.gamepad.x)
            TURN_PID_CONSTANTS.kP += .0001;
        else if (gamepad.gamepad.b)
            TURN_PID_CONSTANTS.kP -= .0001;
    }

    /**
     * Controls translation and rotation, constantly supplying vectors for the swiveling tasks.
     */
    private class SwerveDriveTask extends SimpleTask
    {
        // Unboxing/boxing slowdown fix.
        double gyroHeading = 0;
        double rotationSpeed;
        Vector2D fieldCentricTranslation;
        double angleOff;

        // Display latency to the drivers.
        double averageUpdateRate = 0;
        long lastRunTime = -1, totalRuns = 0;

        @Override
        protected long onContinueTask() throws InterruptedException
        {
            if (gamepad != null)
                updateTeleopInstructions();

            // Get current gyro val.
            gyroHeading = gyro.z();

            // Find the least heading between the gyro and the current heading.
            angleOff = (Vector2D.clampAngle(desiredHeading - gyroHeading) + 180) % 360 - 180;
            angleOff = angleOff < -180 ? angleOff + 360 : angleOff;

            // Figure out the actual translation vector for swerve wheels based on gyro value.
            fieldCentricTranslation = desiredMovement.rotateBy(-gyroHeading);

            // Don't bother trying to be more accurate than 8 degrees while turning.
            rotationSpeed = -pidController.calculatePIDCorrection(angleOff);

            /*
             * Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
             * Note that I'm scaling these up by 100 to convert to cm/s from encoder ticks/s
             */
            frontLeft.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[0]).add(fieldCentricTranslation).multiply(100));
            backLeft.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[1]).add(fieldCentricTranslation).multiply(100));
            backRight.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[2]).add(fieldCentricTranslation).multiply(100));
            frontRight.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[3]).add(fieldCentricTranslation).multiply(100));

            // Figure out latency (not really required persay but useful to see).
            if (lastRunTime != -1)
            {
                averageUpdateRate = (averageUpdateRate * totalRuns + (System.currentTimeMillis() - lastRunTime)) / (totalRuns + 1);
                totalRuns++;
            }
            lastRunTime = System.currentTimeMillis();

            // Write some information to the telemetry console.
            swerveConsole.write(
                    "Current Heading: " + gyroHeading,
                    "Desired Angle: " + desiredHeading,
                    "Rotation Speed: " + rotationSpeed,
                    "Translation Vector: " + desiredMovement.toString(Vector2D.VectorCoordinates.POLAR),
                    "Average update rate: " + averageUpdateRate,
                    "PID kP: " + TURN_PID_CONSTANTS.kP,
                    "PID kD: " + TURN_PID_CONSTANTS.kD
            );

            // Check to see whether it's okay to start moving by observing the state of all wheels.  .
            if (
                    frontLeft.atAcceptableSwivelOrientation() &&
                    frontRight.atAcceptableSwivelOrientation() &&
                    backLeft.atAcceptableSwivelOrientation() &&
                    backRight.atAcceptableSwivelOrientation())
            {
                frontLeft.setDrivingState(true);
                frontRight.setDrivingState(true);
                backLeft.setDrivingState(true);
                backRight.setDrivingState(true);
            } else {
                frontLeft.setDrivingState(false);
                frontRight.setDrivingState(false);
                backLeft.setDrivingState(false);
                backRight.setDrivingState(false);
            }

            return 40;
        }
    }

}
