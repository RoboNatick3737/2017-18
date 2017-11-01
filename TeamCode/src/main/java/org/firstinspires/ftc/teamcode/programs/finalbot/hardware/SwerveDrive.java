package org.firstinspires.ftc.teamcode.programs.finalbot.hardware;

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
        double rotationSpeedCoefficient;

        @Override
        protected long onContinueTask() throws InterruptedException {
            // Figure out the direction which we will be rotating based on the rotation vector for input.
            currentHeading = Vector2D.polar(1, gyro.z());

            // Figure out the actual translation vector for swerve wheels based on gyro value.
            Vector2D fieldCentricTranslation = desiredMovement.rotateBy(-currentHeading.angle);

            // Only try to rotate when we aren't super close to the value we should be working to accomplish already.
            double angleOff = currentHeading.leastAngleTo(desiredHeading);

            // Change the power of the turn speed depending on our distance from the desired heading.  Soon causes turn vector to be zero, allowing movement free of turning to occur.
            rotationSpeedCoefficient = 0;
            if (Math.abs(angleOff) > 8) // Don't try to turn if we're close enough in the range.
                rotationSpeedCoefficient = -1 * Math.signum(angleOff) * Range.clip((.0015 * Math.pow(angleOff, 2)), 0, 1);

            // Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
            frontLeft.setVectorTarget(
                    Vector2D.polar(1, ROBOT_PHI - 90)
                            .multiply(rotationSpeedCoefficient)
                            .add(fieldCentricTranslation));
            backLeft.setVectorTarget(
                    Vector2D.polar(1, (180 - ROBOT_PHI) - 90)
                            .multiply(rotationSpeedCoefficient)
                            .add(fieldCentricTranslation));
            backRight.setVectorTarget(
                    Vector2D.polar(1, (180 + ROBOT_PHI) - 90)
                            .multiply(rotationSpeedCoefficient)
                            .add(fieldCentricTranslation));
            frontRight.setVectorTarget(
                    Vector2D.polar(1, (360 - ROBOT_PHI) - 90)
                            .multiply(rotationSpeedCoefficient)
                            .add(fieldCentricTranslation));

            // Write some information to the telemetry console.
            swerveConsole.write(
                    "Current heading: " + currentHeading.angle,
                    "Desired angle: " + desiredHeading.angle,
                    "Rotation speed coeff: " + rotationSpeedCoefficient
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
