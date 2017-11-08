package org.firstinspires.ftc.teamcode.programs.finalbot.hardware;

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
    private Vector2D desiredHeading = Vector2D.polar(1, 0), desiredMovement = Vector2D.ZERO;

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
     * @param newlyDesiredRotation the new orientation to align to.
     */
    public void setDesiredHeading(Vector2D newlyDesiredRotation)
    {
        this.desiredHeading = newlyDesiredRotation;
    }

    /**
     * Sets new desired translation.
     * @param newlyDesiredMovement the new desired movement vector.
     *
     */
    public void setDesiredMovement(Vector2D newlyDesiredMovement)
    {
        this.desiredMovement = newlyDesiredMovement;
    }

    /**
     * Controls translation and rotation, constantly supplying vectors for the swiveling tasks.
     */
    private class SwerveDriveTask extends SimpleTask
    {
        // Unboxing/boxing slowdown fix.
        Vector2D currentHeading;
        double rotationSpeed;
        Vector2D fieldCentricTranslation;
        double angleOff;

        // Teleop stuff
        Vector2D desiredRotation, desiredMovement;

        @Override
        protected long onContinueTask() throws InterruptedException
        {
            // Teleop instructions (only executed if provided).
            if (gamepad != null)
            {
                desiredRotation = Vector2D.rectangular(gamepad.left_stick_x, -gamepad.left_stick_y).rotateBy(-90);
                desiredMovement = Vector2D.rectangular(gamepad.right_stick_x, -gamepad.right_stick_y).rotateBy(-90);

                if (desiredRotation.magnitude() < .05)
                    desiredRotation = null;

                if (desiredMovement.magnitude() < .05)
                    desiredMovement = Vector2D.ZERO;

                // Rotate by -90 in order to make forward facing zero.
                setDesiredHeading(desiredRotation);
                setDesiredMovement(desiredMovement);

                if (gamepad.a)
                    gyro.calibrate();
            }

            // Figure out the direction which we will be rotating based on the rotation vector for input.
            currentHeading = Vector2D.polar(1, gyro.z());

            // Figure out the actual translation vector for swerve wheels based on gyro value.
            fieldCentricTranslation = desiredMovement.rotateBy(-currentHeading.angle());

            // Only try to rotate when we aren't super close to the value we should be working to accomplish already.
            angleOff = currentHeading.leastAngleTo(desiredHeading);

            // Change the power of the turn speed depending on our distance from the desired heading.  Soon causes turn vector to be zero, allowing movement free of turning to occur.
            rotationSpeed = 0;
            if (Math.abs(angleOff) > 8) // Don't try to turn if we're close enough in the range.
                rotationSpeed = -1 * Range.clip((.0015 * angleOff), -1, 1);

            // Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
            frontLeft.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[0]).add(fieldCentricTranslation));
            backLeft.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[1]).add(fieldCentricTranslation));
            backRight.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[2]).add(fieldCentricTranslation));
            frontRight.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[3]).add(fieldCentricTranslation));

            // Write some information to the telemetry console.
            swerveConsole.write(
                    "Current heading: " + currentHeading.angle(),
                    "Desired angle: " + desiredHeading.angle(),
                    "Rotation speed coeff: " + rotationSpeed
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
