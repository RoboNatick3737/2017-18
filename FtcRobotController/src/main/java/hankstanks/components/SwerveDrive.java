package hankstanks.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import hankextensions.hardware.Gyro;
import hankextensions.threading.SimpleTask;
import hankextensions.threading.SimpleTaskPackage;

public class SwerveDrive
{
    private final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private final double ROBOT_PHI = Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH); // Will be 45 degrees with perfect square dimensions.

    private SwerveWheel frontLeft, frontRight, backLeft, backRight;
    private Gyro gyro;

    /**
     * This task package runs on a different thread, continually updating the vectors for each swerve wheel, while also
     * running the tasks which update the orientation of the swerve wheel.
     *
     * They're all in the same package in order to avoid using a bunch of tasks on solely this.
     */
    private SimpleTaskPackage drivingTasks;

    // The autonomous swerve drive constructor.
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
        drivingTasks.start();
    }
    // The teleop swerve drive constructor.
    public SwerveDrive(Gyro gyro, SwerveWheel frontLeft, SwerveWheel frontRight, SwerveWheel backLeft, SwerveWheel backRight, Gamepad controller)
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
        drivingTasks.add(new JoystickNavigationTask(controller)); // Controls the swerve drive in teleop.
        drivingTasks.start();
    }

    /**
     * Controls translation and rotation, constantly supplying vectors for the swiveling tasks.
     */
    private class JoystickNavigationTask extends SimpleTask
    {
        private Gamepad controller;
        private Vector2D desiredRotation;// Controlled by left joystick, front of the robot is 90 degrees.  Vector because mag is important (changes speed of rotation).
        private Vector2D desiredTranslation; // Controlled by right joystick, up means move forward.

        public JoystickNavigationTask(Gamepad controller)
        {
            this.controller = controller;
        }

        @Override
        protected long onContinueTask() throws InterruptedException
        {
            // Recalculate rotation.
            Vector2D rotationVector = new Vector2D(controller.left_stick_x, controller.left_stick_y);
            if (rotationVector.magnitude() < 0.1) rotationVector = desiredRotation;
            else desiredRotation = rotationVector;

            // Recalculate translation.
            Vector2D translationVector = new Vector2D(controller.right_stick_x, controller.right_stick_y);
            if (rotationVector.magnitude() < 0.1) translationVector = desiredTranslation;
            else desiredTranslation = translationVector;


            // Figure out the direction which we will be rotating based on the rotation vector for input.
            double currentAngle = gyro.heading();
            double desiredAngle = rotationVector.angle();

            // Only try to rotate when we aren't super close to the value we should be working to accomplish already.
            if (currentAngle - desiredAngle > 5)
            {
                // Don't rotate as quickly as we approach the angle we want to stop at.
                double rotationSpeedCoefficient = Range.clip(Math.abs(0.6 + (currentAngle - desiredAngle) / 360.0), 0, 1);
                Vector2D modifiedRotationVector = rotationVector.multiply(rotationSpeedCoefficient);

                // Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
                frontLeft.setVectorTarget(modifiedRotationVector.orientToAngle(ROBOT_PHI + currentAngle).add(translationVector).unit());
                frontRight.setVectorTarget(modifiedRotationVector.orientToAngle((180 - ROBOT_PHI) + currentAngle).add(translationVector).unit());
                backLeft.setVectorTarget(modifiedRotationVector.orientToAngle((180 + ROBOT_PHI) + currentAngle).add(translationVector).unit());
                backRight.setVectorTarget(modifiedRotationVector.orientToAngle((360 - ROBOT_PHI) + currentAngle).add(translationVector).unit());
            }
            else
            {
                Vector2D translationUnitVector = translationVector.unit();

                frontLeft.setVectorTarget(translationUnitVector);
                frontRight.setVectorTarget(translationUnitVector);
                backLeft.setVectorTarget(translationUnitVector);
                backRight.setVectorTarget(translationUnitVector);
            }

            return 40;
        }
    }
}
