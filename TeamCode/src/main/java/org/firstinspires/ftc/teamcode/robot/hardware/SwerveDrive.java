package org.firstinspires.ftc.teamcode.robot.hardware;

import android.support.annotation.NonNull;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.Flow;
import com.makiah.makiahsandroidlib.threading.ScheduledTask;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import hankextensions.EnhancedOpMode;
import hankextensions.input.HTGamepad;
import hankextensions.phonesensors.Gyro;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.structs.VariableVector2D;
import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.structs.pid.PIDController;
import hankextensions.structs.Vector2D;

/**
 * The SwerveDrive contains 4 SwerveModule instances to which a number of vectors are specified
 * in order to determine the direction in which movement will occur.
 */
public class SwerveDrive extends ScheduledTask
{
    // region Physical Drive Constants
    private static final double MAX_SWERVE_SPEED_CM_S = 130; // modified over teleop duration.
    private static final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH)); // Will be 45 degrees with perfect square dimensions.
    private static final double[] WHEEL_ORIENTATIONS = {ROBOT_PHI - 90, (180 - ROBOT_PHI) - 90, (180 + ROBOT_PHI) - 90, (360 - ROBOT_PHI) - 90};
    private static final PIDConstants TURN_PID_CONSTANTS = new PIDConstants(.005, 0, 0, 5, 40000000);
    // endregion

    // region Initialization
    // The SwerveModule instances which constitute the swerve drive: frontLeft, backLeft, backRight, frontRight respectively.
    public final SwerveModule[] swerveModules = new SwerveModule[4];

    // Robot reference (for gyro and such.
    private final Robot robot;

    // For turning the robot.
    private final PIDController pidController;

    // Required for operation of the driving tasks.
    private final ScheduledTaskPackage swerveUpdatePackage;

    // The logger for when data needs to be displayed to the drivers.
    private final ProcessConsole swerveConsole;

    /**
     * Constructor, starts the alignment threads and such.
     * @param robot        Contains a gyro and the control method for the bot.
     * @param frontLeft    The front left swerve module
     * @param frontRight   The front right swerve module
     * @param backLeft     The back left swerve module.
     * @param backRight    The back right swerve module.
     */
    public SwerveDrive(Robot robot, SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight)
    {
        // Robot reference
        this.robot = robot;

        // The swerve wheels.
        this.swerveModules[0] = frontLeft;
        this.swerveModules[1] = backLeft;
        this.swerveModules[2] = backRight;
        this.swerveModules[3] = frontRight;

        // Initialize the task package regardless we need it atm, better to have it and skip the initialization sequence.
        swerveUpdatePackage = new ScheduledTaskPackage(EnhancedOpMode.instance, "Swerve Turn Alignments",
                this, frontLeft, frontRight, backLeft, backRight);

        // For turning the drive.
        pidController = new PIDController(TURN_PID_CONSTANTS);

        swerveConsole = LoggingBase.instance.newProcessConsole("Swerve Console");
    }
    // endregion

    // region Vector Stuff
    // Constantly shifting in autonomous and teleop.
    private Vector2D desiredMovement = Vector2D.ZERO;
    private double desiredHeading = 0;
    //endregion

    // region Control Methods
    public enum ControlMethod { FIELD_CENTRIC, TANK_DRIVE }
    private ControlMethod controlMethod = ControlMethod.FIELD_CENTRIC;
    public void setControlMethod(ControlMethod controlMethod)
    {
        this.controlMethod = controlMethod;
    }
    public ControlMethod getControlMethod()
    {
        return controlMethod;
    }

    /**
     * This is where pretty much all the work for the swerve DRIVE calculations take place
     * (calculating the vectors to which the wheels should align), but the wheels have their own
     * update methods to actually change their orientation.
     */
    private void updateSwerveDriveFieldCentric() throws InterruptedException
    {
        if (robot.controlMode == Robot.ControlMode.TELEOP)
        {
            // Rotate by -90 in order to make forward facing zero.
            Vector2D joystickDesiredRotation = HTGamepad.CONTROLLER1.rightJoystick();
            Vector2D joystickDesiredMovement = HTGamepad.CONTROLLER1.leftJoystick();

            // Use the left joystick for rotation unless nothing is supplied, in which case check the DPAD.
            if (joystickDesiredRotation.magnitude > .0005)
                setDesiredHeading(joystickDesiredRotation.angle);

            if (joystickDesiredMovement.magnitude > .0005)
                setDesiredMovement(joystickDesiredMovement);
            else
                setDesiredMovement(Vector2D.ZERO);

            // Upon tapping white, calibrate the gyro
            if (HTGamepad.CONTROLLER1.gamepad.y)
            {
                try
                {
                    robot.gyro.zero();
                }
                catch (InterruptedException e)
                {
                    return;
                }
            }

            // Fine tuned adjustments.
            if (HTGamepad.CONTROLLER1.gamepad.left_trigger > 0.1 || HTGamepad.CONTROLLER1.gamepad.right_trigger > 0.1)
            {
                this.desiredHeading += 5 * (HTGamepad.CONTROLLER1.gamepad.left_trigger - HTGamepad.CONTROLLER1.gamepad.right_trigger);
                this.desiredHeading = Vector2D.clampAngle(this.desiredHeading);
            }
        }

        // Get current gyro val.
        double gyroHeading = robot.gyro.getHeading();

        // Find the least heading between the gyro and the current heading.
        double angleOff = (Vector2D.clampAngle(desiredHeading - gyroHeading) + 180) % 360 - 180;
        angleOff = angleOff < -180 ? angleOff + 360 : angleOff;

        // Figure out the actual translation vector for swerve wheels based on gyro value.
        Vector2D fieldCentricTranslation = desiredMovement.rotateBy(-gyroHeading);

        // Don't bother trying to be more accurate than 8 degrees while turning.
        double rotationSpeed = -pidController.calculatePIDCorrection(angleOff);

        // Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
        for (int i = 0; i < swerveModules.length; i++)
            swerveModules[i].setVectorTarget(
                    Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[i]).add(fieldCentricTranslation).multiply(MAX_SWERVE_SPEED_CM_S));

        // Check to see whether it's okay to start moving by observing the state of all wheels.
        boolean drivingCanStart = true;
        for (SwerveModule wheel : swerveModules)
        {
            if (!wheel.atAcceptableSwivelOrientation())
            {
                drivingCanStart = false;
                break;
            }
        }
        for (SwerveModule wheel : swerveModules)
            wheel.setDrivingState(drivingCanStart);


        // Write some information to the telemetry console.
        swerveConsole.write(
                "Current Heading: " + gyroHeading,
                "Desired Angle: " + desiredHeading,
                "Rotation Speed: " + rotationSpeed,
                "Translation Vector: " + desiredMovement.toString(Vector2D.VectorCoordinates.POLAR),
                "PID kP: " + TURN_PID_CONSTANTS.kP,
                "PID kD: " + TURN_PID_CONSTANTS.kD,
                "Magnitude: " + fieldCentricTranslation.magnitude,
                "Driving acceptable: " + drivingCanStart
        );
    }

    /**
     * Simple swerve drive control which doesn't include any sort of gyroscope adjustment.
     */
    private void updateSwerveDriveTankDrive() throws InterruptedException
    {
        double rotationSpeed = 0;

        // Receive controller input.
        if (robot.controlMode == Robot.ControlMode.TELEOP)
        {
            desiredMovement = HTGamepad.CONTROLLER1.leftJoystick();
            if (desiredMovement.magnitude < .0005)
                desiredMovement = Vector2D.ZERO;

            rotationSpeed = HTGamepad.CONTROLLER1.gamepad.right_stick_x;
        }

        // Set vector targets for wheels.
        for (int i = 0; i < swerveModules.length; i++)
            swerveModules[i].setVectorTarget(
                    Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[i]).add(desiredMovement).multiply(MAX_SWERVE_SPEED_CM_S));
    }
    // endregion

    /**
     * Shifts the current swerve control system between synchronous and asynchronous.
     */
    public void setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode updateMode)
    {
        swerveUpdatePackage.setUpdateMode(updateMode);

        if (updateMode == ScheduledTaskPackage.ScheduledUpdateMode.ASYNCHRONOUS)
            swerveUpdatePackage.run();
    }

    /**
     * Synchronous version of the async task thread.
     */
    public void synchronousUpdate() throws InterruptedException
    {
        // Only updates if the control system has been set to synchronous.
        swerveUpdatePackage.synchronousUpdate();
    }

    /**
     * A scheduled task to ensure swerve consistency.
     */
    @Override
    protected long onContinueTask() throws InterruptedException
    {
        if (controlMethod == ControlMethod.FIELD_CENTRIC)
            updateSwerveDriveFieldCentric();
        else if (controlMethod == ControlMethod.TANK_DRIVE)
            updateSwerveDriveTankDrive();

        return 40;
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

    // region Autonomous Methods
    /**
     * Represents the encoder positions of each of the four drive motor encoders.
     */
    public double[] currentDrivePosition()
    {
        double[] toReturn = new double[swerveModules.length];
        for (int i = 0; i < toReturn.length; i++)
            toReturn[i] = swerveModules[i].driveMotor.currentDistanceMoved();
        return toReturn;
    }

    /**
     * Drives a certain distance with a variable heading/power.
     * @param direction represents the direction and speed of movement, with current distance
     *                  moved as the parameter.
     * @param distance  the distance in inches to move.
     */
    public void driveDistance(VariableVector2D direction, double distance, Flow flow) throws InterruptedException
    {
        // Represents distance driven by each module.
        double[] cumulativeOffsets = new double[swerveModules.length];

        double[] lastPosition = currentDrivePosition();
        boolean canStop = false; // becomes true once the average offset reaches the requested distance.
        while (!canStop)
        {
            double[] currentPosition = currentDrivePosition();

            // Determine how far we've moved and add that distance to the cumulative offsets.
            for (int i = 0; i < currentPosition.length; i++)
                cumulativeOffsets[i] += Math.abs(currentPosition[i] - lastPosition[i]);

            // New anchor.
            lastPosition = currentPosition;

            // Figure out the average offset and thus whether we can stop.
            double avgOffset = 0;
            for (double offset : cumulativeOffsets)
                avgOffset += offset;
            avgOffset /= cumulativeOffsets.length;
            canStop = avgOffset >= distance;

            // Keep updating unless we can stop.
            if (!canStop)
            {
                setDesiredMovement(direction.getVector(avgOffset));

                if (swerveUpdatePackage.getUpdateMode() == ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS)
                    synchronousUpdate();
            }

            flow.yield();
        }

        stop();
    }
    public void driveDistance(Vector2D direction, double distance, Flow flow) throws InterruptedException
    {
        driveDistance(VariableVector2D.from(direction), distance, flow);
    }
    // endregion

    /**
     * Stops all SwerveWheels.
     */
    public void stop()
    {
        for (SwerveModule wheel : swerveModules)
            wheel.stopWheel();
    }
}
