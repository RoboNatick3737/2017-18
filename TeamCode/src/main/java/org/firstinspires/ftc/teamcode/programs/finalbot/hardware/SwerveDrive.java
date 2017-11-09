package org.firstinspires.ftc.teamcode.programs.finalbot.hardware;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import hankextensions.logging.Log;
import hankextensions.logging.ProcessConsole;
import hankextensions.phonesensors.Gyro;

import org.firstinspires.ftc.teamcode.structs.Vector2D;

import hankextensions.threading.SimpleTask;
import hankextensions.threading.SimpleTaskPackage;

public class SwerveDrive
{
    // Physical robot constants.
    private static final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH)); // Will be 45 degrees with perfect square dimensions.
    private static final double[] WHEEL_ORIENTATIONS = {ROBOT_PHI - 90, (180 - ROBOT_PHI) - 90, (180 + ROBOT_PHI) - 90, (360 - ROBOT_PHI) - 90};

    // Instance specific components.
    private final SwerveWheel frontLeft, frontRight, backLeft, backRight;
    public final Gyro gyro; // Public because teleop can manually reset.

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
    private Gamepad gamepad;

    // The swerve drive constructor, starts all swerve wheel alignment threads.
    public SwerveDrive(Gyro gyro, SwerveWheel frontLeft, SwerveWheel frontRight, SwerveWheel backLeft, SwerveWheel backRight) throws InterruptedException
    {
        this.gyro = gyro;

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        // Construct the task package which will update all of the turning for the swerve motors.
        drivingTasks = new SimpleTaskPackage("Swerve Turn Alignments");
        drivingTasks.add(this.frontLeft.swivelTask);
        drivingTasks.add(this.frontRight.swivelTask);
        drivingTasks.add(this.backLeft.swivelTask);
        drivingTasks.add(this.backRight.swivelTask);
        drivingTasks.add(new SwerveDriveTask());

        swerveConsole = Log.instance.newProcessConsole("Swerve Console");

        Log.instance.lines("Wheel orientations: " + WHEEL_ORIENTATIONS[0] + ", " + WHEEL_ORIENTATIONS[1] + ", " + WHEEL_ORIENTATIONS[2] + ", " + WHEEL_ORIENTATIONS[3]);

        drivingTasks.start();
    }

    /**
     * Tells the swerve drive how to accomplish teleop.
     * @param gamepad the gamepad which provides instructions.
     */
    public void provideGamepad(Gamepad gamepad)
    {
        this.gamepad = gamepad;
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
        Vector2D joystickDesiredRotation = Vector2D.rectangular(gamepad.left_stick_x, -gamepad.left_stick_y).rotateBy(-90);
        Vector2D joystickDesiredMovement = Vector2D.rectangular(gamepad.right_stick_x, -gamepad.right_stick_y).rotateBy(-90);

        if (joystickDesiredRotation.magnitude > .05)
            setDesiredHeading(joystickDesiredRotation.angle);

        if (joystickDesiredMovement.magnitude > .05)
            setDesiredMovement(joystickDesiredMovement);
        else
            setDesiredMovement(Vector2D.ZERO);

        if (gamepad.a)
        {
            gyro.calibrate();
            setDesiredHeading(0);
        }
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
            rotationSpeed = Math.abs(angleOff) > 15 ? -1 * Math.signum(.00005 * angleOff) : 0;

            /*
             * Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
             * Note that I'm scaling these up by 100 to convert to cm/s from encoder ticks/s
             */
            frontLeft.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[0]).add(fieldCentricTranslation).multiply(100));
            backLeft.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[1]).add(fieldCentricTranslation).multiply(100));
            backRight.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[2]).add(fieldCentricTranslation).multiply(100));
            frontRight.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[3]).add(fieldCentricTranslation).multiply(70));

            // Write some information to the telemetry console.
            swerveConsole.write(
                    "Current Heading: " + gyroHeading,
                    "Desired Angle: " + desiredHeading,
                    "Rotation Speed: " + rotationSpeed,
                    "Translation Vector: " + desiredMovement.toString(Vector2D.VectorCoordinates.POLAR)
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

            return 100;
        }
    }

}
