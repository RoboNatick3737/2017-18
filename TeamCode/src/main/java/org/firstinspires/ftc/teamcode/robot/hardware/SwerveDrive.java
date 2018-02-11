package org.firstinspires.ftc.teamcode.robot.hardware;

import android.support.annotation.NonNull;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.Flow;
import com.makiah.makiahsandroidlib.threading.ScheduledTask;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import hankextensions.EnhancedOpMode;
import hankextensions.input.HTGamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.structs.Function;
import org.firstinspires.ftc.teamcode.structs.ParametrizedVector;
import org.firstinspires.ftc.teamcode.structs.SingleParameterRunnable;

import hankextensions.structs.Vector2D;

/**
 * The SwerveDrive contains 4 SwerveModule instances to which a number of vectors are specified
 * in order to determine the direction in which movement will occur.
 */
public class SwerveDrive extends ScheduledTask
{
    // region Physical Drive OpModeDisplayGroups
    private static final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH)); // Will be 45 degrees with perfect square dimensions.
    private static final double[] WHEEL_ORIENTATIONS = {ROBOT_PHI - 90, (180 - ROBOT_PHI) - 90, (180 + ROBOT_PHI) - 90, (360 - ROBOT_PHI) - 90};
//    private static final PIDController FIELD_CENTRIC_TURN_CONTROLLER = new PIDController(.005, 0, 0, 5, PIDController.TimeUnits.MILLISECONDS, 40, -1000, 1000);
    private static Function FIELD_CENTRIC_TURN_CONTROLLER = new Function() {
        @Override
        public double value(double input) {
            return .008 * input;
        }
    };
    // endregion

    // region Initialization
    // The SwerveModule instances which constitute the swerve drive: frontLeft, backLeft, backRight, frontRight respectively.
    public final SwerveModule[] swerveModules;

    // Robot reference (for gyro and such.
    private final Robot robot;

    // Required for operation of the driving tasks.
    private final ScheduledTaskPackage swerveUpdatePackage;

    // The logger for when data needs to be displayed to the drivers.
    private final ProcessConsole swerveConsole;

    // region Fast/Slow Mode
    // The speed of the swerve drive.
    public enum SwerveSpeedMode
    {
        FAST (75),
        SLOW (22);

        public final double speed;
        SwerveSpeedMode(double speed)
        {
            this.speed = speed;
        }
    }
    private SwerveSpeedMode swerveSpeedMode = SwerveSpeedMode.FAST;
    public void setSwerveSpeedMode(SwerveSpeedMode mode)
    {
        this.swerveSpeedMode = mode;
    }
    public SwerveSpeedMode getSwerveSpeedMode()
    {
        return swerveSpeedMode;
    }
    //endregion

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
        this(robot, new SwerveModule[]{frontLeft, backLeft, backRight, frontRight});
    }

    /**
     * Constructor, starts the alignment threads and such.
     * @param robot        Contains a gyro and the control method for the bot.
     * @param modules      The swerve modules (in an array duh)
     */
    public SwerveDrive(Robot robot, SwerveModule[] modules)
    {
        // Robot reference
        this.robot = robot;

        // The swerve wheels.
        this.swerveModules = modules;

        if (robot.controlMode == Robot.ControlMode.AUTONOMOUS)
        {
            // Initialize the task package regardless we need it atm, better to have it and skip the initialization sequence.
            swerveUpdatePackage = new ScheduledTaskPackage(EnhancedOpMode.instance, "Swerve Turn Alignments",
                    this, this.swerveModules[0], this.swerveModules[1], this.swerveModules[2], this.swerveModules[3],
                    this.swerveModules[0].driveMotor, this.swerveModules[1].driveMotor, this.swerveModules[2].driveMotor, this.swerveModules[3].driveMotor);
        }

        // Teleop mode is weird... we don't use drive PID.
        else
        {
            // Initialize the task package regardless we need it atm, better to have it and skip the initialization sequence.
            swerveUpdatePackage = new ScheduledTaskPackage(EnhancedOpMode.instance, "Swerve Turn Alignments",
                    this, this.swerveModules[0], this.swerveModules[1], this.swerveModules[2], this.swerveModules[3]);

            for (SwerveModule module : modules)
                module.setEnableDrivePID(false);
        }

        swerveConsole = LoggingBase.instance.newProcessConsole("Swerve Console");
    }
    // endregion

    // region Control Methods
    private Vector2D desiredMovement = Vector2D.ZERO;
    private double desiredHeading = 0;

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
        double rotationSpeed = FIELD_CENTRIC_TURN_CONTROLLER.value(-angleOff);

        // Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
        for (int i = 0; i < swerveModules.length; i++)
            swerveModules[i].setVectorTarget(
                    Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[i]).add(fieldCentricTranslation)
                            .multiply(swerveSpeedMode.speed)); // whether to be fast/slow

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
                    Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[i]).add(desiredMovement)
                            .multiply(swerveSpeedMode.speed)); // whether to be fast/slow

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
     * @param runnable  Optional parameter to run every loop.
     */
    public void driveDistance(ParametrizedVector direction, double distance, SingleParameterRunnable runnable, Flow flow) throws InterruptedException
    {
        if (distance <= 0)
            return;

        ProcessConsole distanceConsole = LoggingBase.instance.newProcessConsole("Distance Drive");

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
                setDesiredMovement(direction.getVector(avgOffset / distance)); // 0 and 1

                distanceConsole.write(
                        "Cumulatives are: " + cumulativeOffsets[0] + " " + cumulativeOffsets[1] + " " + cumulativeOffsets[2] + " " + cumulativeOffsets[3],
                        "Distances are " + currentPosition[0] + " " + currentPosition[1] + " " + currentPosition[2] + " " + currentPosition[3]);

                if (swerveUpdatePackage.getUpdateMode() == ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS)
                    synchronousUpdate();

                if (runnable != null)
                    runnable.run(avgOffset / distance);
            }

            flow.yield();
        }

        stop();
        distanceConsole.destroy();
    }
    public void driveDistance(Vector2D direction, double distance, Flow flow) throws InterruptedException
    {
        driveDistance(ParametrizedVector.from(direction), distance, null, flow);
    }

    /**
     * Drives a certain time with a variable heading/power
     * @param direction The direction to drive
     * @param driveTime the ms to drive
     * @param runnable runnable to run every loop
     * @param flow When to exit
     */
    public void driveTime(ParametrizedVector direction, long driveTime, SingleParameterRunnable runnable, Flow flow) throws InterruptedException
    {
        if (driveTime <= 0)
            return;

        ProcessConsole distanceConsole = LoggingBase.instance.newProcessConsole("Timed Drive");

        long startTime = System.currentTimeMillis();
        long elapsedTime = 0;
        while (elapsedTime < driveTime)
        {
            elapsedTime = System.currentTimeMillis() - startTime;

            setDesiredMovement(direction.getVector(elapsedTime / driveTime));
            distanceConsole.write("Remaining: " + (driveTime - (System.currentTimeMillis() - startTime) + "ms"));
            if (swerveUpdatePackage.getUpdateMode() == ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS)
                synchronousUpdate();

            // Shortcut, callers can provide anonymous methods here.
            if (runnable != null)
                runnable.run(elapsedTime / driveTime);

            flow.yield();
        }

        stop();
        distanceConsole.destroy();
    }
    public void driveTime(Vector2D direction, long msDrive, Flow flow) throws InterruptedException
    {
        driveTime(ParametrizedVector.from(direction), msDrive, null, flow);
    }

    /**
     * Orients all SwerveModules to a given vector with some degree of certainty (for auto)
     */
    public void orientSwerveModules(Vector2D orientationVector, double precisionRequired, long msMax, Flow flow) throws InterruptedException
    {
        for (int i = 0; i < swerveModules.length; i++)
            swerveModules[i].setVectorTarget(orientationVector);

        orientModules(precisionRequired, msMax, flow);
    }

    /**
     * Orient all swivel modules for rotation.
     */
    public void orientSwerveModulesForRotation(double precisionRequired, long msMax, Flow flow) throws InterruptedException
    {
        for (int i = 0; i < swerveModules.length; i++)
            swerveModules[i].setVectorTarget(Vector2D.polar(1, WHEEL_ORIENTATIONS[i]));

        orientModules(precisionRequired, msMax, flow);
    }

    /**
     * Private method which actually takes care of the updating bit.
     *
     * TODO include asynchronous mode
     */
    private void orientModules(double precisionRequired, long msMax, Flow flow) throws InterruptedException
    {
        // Have to make a separate package for just swiveling.
        ScheduledTaskPackage updatePackage = new ScheduledTaskPackage(
                flow.parent,
                "Orienting",
                swerveModules[0], swerveModules[1], swerveModules[2], swerveModules[3]);
        updatePackage.setUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < msMax)
        {
            // Otherwise update
            updatePackage.synchronousUpdate();

            // Whether orientations need to keep being updated.
            boolean orientationsAreGood = true;
            for (SwerveModule module : swerveModules)
            {
                if (Math.abs(module.getAngleLeftToTurn()) > precisionRequired)
                {
                    orientationsAreGood = false;
                    break;
                }
            }

            if (orientationsAreGood)
                break;

            flow.yield();
        }

        // Stop updating.
        updatePackage.stop();

        // Stop all modules.
        stop();
    }
    // endregion

    /**
     * Stops all SwerveWheels and sets the current desired movement to zero, as well as desired
     * heading to current heading.
     */
    public void stop()
    {
        for (SwerveModule wheel : swerveModules)
            wheel.stopWheel();

        setDesiredMovement(Vector2D.ZERO);
        setDesiredHeading(0);
    }
}
