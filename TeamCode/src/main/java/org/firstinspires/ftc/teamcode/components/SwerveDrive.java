package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.sdkextensions.hardware.Gyro;
import org.firstinspires.ftc.teamcode.sdkextensions.threading.SimpleTask;
import org.firstinspires.ftc.teamcode.sdkextensions.threading.SimpleTaskPackage;

public class SwerveDrive
{
    private SwerveMotor frontLeft, frontRight, backLeft, backRight;
    private Gyro gyro;

    private SimpleTaskPackage drivingTasks;

    // The autonomous swerve drive constructor.
    public SwerveDrive(Gyro gyro, SwerveMotor frontLeft, SwerveMotor frontRight, SwerveMotor backLeft, SwerveMotor backRight)
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
    public SwerveDrive(Gyro gyro, SwerveMotor frontLeft, SwerveMotor frontRight, SwerveMotor backLeft, SwerveMotor backRight, Gamepad controller)
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

        public JoystickNavigationTask(Gamepad controller)
        {
            this.controller = controller;
        }

        @Override
        protected long onContinueTask() throws InterruptedException
        {
            Vector2D rotation = new Vector2D(controller.left_stick_x, controller.left_stick_y);
            Vector2D translation = new Vector2D(controller.right_stick_x, controller.right_stick_y);

            // TODO: All calculations for adding rotation and translation will go here.

            return 40;
        }
    }
}
