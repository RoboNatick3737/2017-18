package org.firstinspires.ftc.teamcode.programs.finalbot.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import hankextensions.logging.Log;
import hankextensions.logging.ProcessConsole;
import hankextensions.phonesensors.Gyro;

import org.firstinspires.ftc.teamcode.structs.LimitAngle;
import org.firstinspires.ftc.teamcode.structs.Vector2D;

import hankextensions.threading.SimpleTask;
import hankextensions.threading.SimpleTaskPackage;

public class SwerveDrive
{
    private final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private final double ROBOT_PHI = Math.toDegrees(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH)); // Will be 45 degrees with perfect square dimensions.

    private SwerveWheel frontLeft, frontRight, backLeft, backRight;
    private Gyro gyro;

    /**
     * This task package runs on a different thread, continually updating the vectors for each swerve wheel, while also
     * running the tasks which update the orientation of the swerve wheel.
     *
     * They're all in the same package in order to avoid using a bunch of tasks on solely this.
     */
    private SimpleTaskPackage drivingTasks;
    private ProcessConsole swerveConsole;

    // The swerve drive constructor, starts all swerve wheel alignment threads.
    public SwerveDrive(Gyro gyro, SwerveWheel frontLeft, SwerveWheel frontRight, SwerveWheel backLeft, SwerveWheel backRight)
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

        swerveConsole = Log.instance.newProcessConsole("Swerve Console");

        drivingTasks.start();
    }


    /// Autonomous swerve drive methods (wheel alignment, driving, etc.). ///
    /**
     * JUST sets the orientation of the wheel.
     * @param vector unit vector for alignment, since mag is pointless
     */
    public void alignWheelsTo(Vector2D vector)
    {
        frontLeft.setVectorTarget(vector);
        frontRight.setVectorTarget(vector);
        backLeft.setVectorTarget(vector);
        backRight.setVectorTarget(vector);

        swerveConsole.write("Set new wheel alignment to " + vector.toString());
    }


    /// Joystick Navigation Stuff ///
    private JoystickNavigationTask joystickNavigationTask = null;
    public void startJoystickControl(Gamepad controller) throws InterruptedException
    {
        if (joystickNavigationTask != null)
                return;

        joystickNavigationTask = new JoystickNavigationTask(controller); // Controls the swerve drive in teleop.
        drivingTasks.add(joystickNavigationTask);
    }

    public void stopJoystickControl()
    {
        if (joystickNavigationTask == null)
            return;

        drivingTasks.remove(joystickNavigationTask);
    }

    /**
     * Controls translation and rotation, constantly supplying vectors for the swiveling tasks.
     */
    private class JoystickNavigationTask extends SimpleTask
    {
        private Gamepad controller;
        private Vector2D desiredRotation = new Vector2D(0, 1);// Controlled by left joystick, front of the robot is 90 degrees.  Vector because mag is important (changes speed of rotation).
        private Vector2D desiredTranslation = new Vector2D(0, 0); // Controlled by right joystick, up means move forward.

        public JoystickNavigationTask(Gamepad controller) throws InterruptedException
        {
            this.controller = controller;
            gyro.zero();
        }

        @Override
        protected long onContinueTask() throws InterruptedException
        {
            // Recalculate rotation.
            Vector2D rotationVector = new Vector2D(controller.left_stick_x, -controller.left_stick_y);
            if (rotationVector.magnitude() < 0.05) rotationVector = desiredRotation;
            else desiredRotation = rotationVector;

            // Recalculate translation.
            Vector2D translationVector = Vector2D.ZERO; //new Vector2D(controller.right_stick_x, -controller.right_stick_y);
            if (translationVector.magnitude() < 0.05) translationVector = desiredTranslation;
            else desiredTranslation = translationVector;


            // Figure out the direction which we will be rotating based on the rotation vector for input.
            LimitAngle currentAngle = new LimitAngle(gyro.z());
            LimitAngle desiredAngle = new LimitAngle(rotationVector.angle() - 90);

            // Only try to rotate when we aren't super close to the value we should be working to accomplish already.
            double angleOff = currentAngle.angleTo(desiredAngle);

            if (Math.abs(angleOff) > 5)
            {
                // Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
                frontLeft.setVectorTarget(
                        rotationVector.orientToAngle(ROBOT_PHI - 90) // only magnitude of rot vector matters.
                                .multiply(Range.clip(Math.signum(angleOff) * (.6 + angleOff / 180.0), -1, 1))
                                .add(translationVector));
                backLeft.setVectorTarget(
                        rotationVector.orientToAngle((180 - ROBOT_PHI) - 90)
                                .multiply(Range.clip(Math.signum(angleOff) * (.6 + angleOff / 180.0), -1, 1))
                                .add(translationVector));
                backRight.setVectorTarget(
                        rotationVector.orientToAngle((180 + ROBOT_PHI) - 90)
                                .multiply(Range.clip(Math.signum(angleOff) * (.6 + angleOff / 180.0), -1, 1))
                                .add(translationVector));
                frontRight.setVectorTarget(
                        rotationVector.orientToAngle((360 - ROBOT_PHI) - 90)
                                .multiply(Range.clip(Math.signum(angleOff) * (.6 + angleOff / 180.0), -1, 1))
                                .add(translationVector));


                swerveConsole.write(
                        "Rotation Input: " + rotationVector.toString(),
                        "Translation Input: " + translationVector.toString(),
                        "Current heading: " + currentAngle.value,
                        "Desired angle: " + desiredAngle.value,
                        "Off from angle: " + angleOff,
                        "PHI: " + ROBOT_PHI
                );
            }
            else
            {
                Vector2D translationUnitVector = translationVector.unit();

                frontLeft.setVectorTarget(translationUnitVector);
                frontRight.setVectorTarget(translationUnitVector);
                backLeft.setVectorTarget(translationUnitVector);
                backRight.setVectorTarget(translationUnitVector);
            }

            // Check to see whether it's okay to start moving (only move if at that state).
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
