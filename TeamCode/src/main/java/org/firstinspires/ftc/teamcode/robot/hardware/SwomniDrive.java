package org.firstinspires.ftc.teamcode.robot.hardware;

import android.support.annotation.NonNull;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.Flow;
import com.makiah.makiahsandroidlib.threading.ScheduledTask;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import hankextensions.EnhancedOpMode;
import hankextensions.input.HTGamepad;

import org.firstinspires.ftc.teamcode.structs.PIDController;
import org.firstinspires.ftc.teamcode.structs.ParametrizedVector;
import org.firstinspires.ftc.teamcode.structs.SingleParameterRunnable;

import hankextensions.phonesensors.Gyro;
import hankextensions.structs.Vector2D;

/**
 * The SwomniDrive contains 4 SwerveModule instances to which a number of vectors are specified
 * in order to determine the direction in which movement will occur.
 */
public class SwomniDrive extends ScheduledTask
{
    private static final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH)); // Will be 45 degrees with perfect square dimensions.
    private static final double[] WHEEL_ORIENTATIONS = {ROBOT_PHI - 90, (180 - ROBOT_PHI) - 90, (180 + ROBOT_PHI) - 90, (360 - ROBOT_PHI) - 90};
    private static final PIDController FIELD_CENTRIC_TURN_CONTROLLER = new PIDController(.008, 0, 0, 5, PIDController.TimeUnits.MILLISECONDS, 40, -1000, 1000);

    // Robot reference (for gyro and such).
    private final Gyro gyro;
    private final EnhancedOpMode.AutoOrTeleop opModeSituation;

    // The SwerveModule instances which constitute the swerve drive: frontLeft, backLeft, backRight, frontRight respectively.
    public final SwerveModule[] swerveModules;

    // Required for operation of the driving tasks.
    private final ScheduledTaskPackage swerveUpdatePackage;
    public void setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode updateMode)
    {
        swerveUpdatePackage.setUpdateMode(updateMode);

        if (updateMode == ScheduledTaskPackage.ScheduledUpdateMode.ASYNCHRONOUS)
            swerveUpdatePackage.run();
    }
    public void synchronousUpdate() throws InterruptedException
    {
        // Only updates if the control system has been set to synchronous.
        swerveUpdatePackage.synchronousUpdate();
    }

    // The logger for when data needs to be displayed to the drivers.
    private final ProcessConsole swerveConsole;

    /**
     * Constructor, starts the alignment threads and such.
     * @param opModeSituation        The part of the program where this occurs.
     * @param modules      The swerve modules (in an array duh)
     */
    public SwomniDrive(EnhancedOpMode.AutoOrTeleop opModeSituation, Gyro gyro, SwerveModule[] modules)
    {
        this.opModeSituation = opModeSituation;
        this.gyro = gyro;

        // The swerve wheels.
        this.swerveModules = modules;

        if (opModeSituation == EnhancedOpMode.AutoOrTeleop.AUTONOMOUS)
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

    // region Fast/Slow Mode
    // The speed of the swerve drive.
    public enum SpeedControl
    {
        FAST (75, 75),
        SLOW (22, 22);

        public final double driveSpeed;
        public final double turnSpeed;
        SpeedControl(double driveSpeed, double turnSpeed)
        {
            this.driveSpeed = driveSpeed;
            this.turnSpeed = turnSpeed;
        }
    }
    private SpeedControl speedControl = SpeedControl.FAST;
    public void setSpeedControl(SpeedControl mode)
    {
        this.speedControl = mode;
    }
    public SpeedControl getSpeedControl()
    {
        return speedControl;
    }
    // endregion

    // region Joystick Drive Methods
    private Vector2D desiredMovement = Vector2D.ZERO;
    private double desiredHeading = 0;

    public enum JoystickControlMethod { FIELD_CENTRIC, ROBOT_CENTRIC}
    private JoystickControlMethod joystickControlMethod = JoystickControlMethod.FIELD_CENTRIC;
    public void setJoystickControlMethod(JoystickControlMethod joystickControlMethod)
    {
        this.joystickControlMethod = joystickControlMethod;
    }
    public JoystickControlMethod getJoystickControlMethod()
    {
        return joystickControlMethod;
    }
    // endregion

    // region Control Modes
    public enum SwomniControlMode
    {
        /**
         * A versatile mode of driving which is quick, but the swiveling involved makes it difficult
         * to make close adjustments near the cryptobox.
         */
        SWERVE_DRIVE,

        /**
         * With swomni drive, omni wheels on each module give us the ability to instantly strafe in
         * holonomic mode instead of waiting for 90 degree module swivels.
         */
        HOLONOMIC,

        /**
         * Sean is a dumpu trucku who actually likes this mode for some weird reason.
         */
        TANK_DRIVE
    }
    private SwomniControlMode swomniControlMode = SwomniControlMode.SWERVE_DRIVE;
    public void setSwomniControlMode(SwomniControlMode swomniControlMode)
    {
        this.swomniControlMode = swomniControlMode;
    }
    // endregion

    private void updateCanDrive()
    {
        // Updates the swerve modules on whether we can drive.
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

    /**
     * A scheduled task to ensure swerve consistency.
     */
    @Override
    protected long onContinueTask() throws InterruptedException
    {
        // Simple code for tank drive.
        if (swomniControlMode == SwomniControlMode.TANK_DRIVE)
        {
            if (opModeSituation == EnhancedOpMode.AutoOrTeleop.TELEOP)
            {
                for (int i = 0 ; i <= 1; i++)
                    swerveModules[i].setVectorTarget(Vector2D.rectangular(HTGamepad.CONTROLLER1.leftJoystick().y, 0));

                for (int i = 2 ; i <= 3; i++)
                    swerveModules[i].setVectorTarget(Vector2D.rectangular(HTGamepad.CONTROLLER1.rightJoystick().y, 0));
            }

            updateCanDrive();

            return 100;
        }

        // Determined based on Field Centric/Robot Centric control mode.
        Vector2D driveVector = Vector2D.ZERO;
        double rotationSpeed = 0;

        // Field centric control
        if (joystickControlMethod == JoystickControlMethod.FIELD_CENTRIC)
        {
            if (opModeSituation == EnhancedOpMode.AutoOrTeleop.TELEOP)
            {
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
                if (HTGamepad.CONTROLLER1.gamepad.y) {
                    try {
                        gyro.zero();
                    } catch (InterruptedException e) {
                        return 100;
                    }
                }

                // Fine tuned adjustments.
                if (HTGamepad.CONTROLLER1.gamepad.left_trigger > 0.1 || HTGamepad.CONTROLLER1.gamepad.right_trigger > 0.1) {
                    this.desiredHeading += 5 * (HTGamepad.CONTROLLER1.gamepad.left_trigger - HTGamepad.CONTROLLER1.gamepad.right_trigger);
                    this.desiredHeading = Vector2D.clampAngle(this.desiredHeading);
                }
            }

            // Get current gyro val.
            double gyroHeading = gyro.getHeading();

            // Find the least heading between the gyro and the current heading.
            double angleOff = (Vector2D.clampAngle(desiredHeading - gyroHeading) + 180) % 360 - 180;
            angleOff = angleOff < -180 ? angleOff + 360 : angleOff;

            // Figure out the actual translation vector for swerve wheels based on gyro value.
            Vector2D fieldCentricTranslation = desiredMovement.rotateBy(-gyroHeading);

            // Don't bother trying to be more accurate than 8 degrees while turning.
            rotationSpeed = FIELD_CENTRIC_TURN_CONTROLLER.value(-angleOff);

            // Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
            driveVector = fieldCentricTranslation.multiply(speedControl.driveSpeed);

            // Write some information to the telemetry console.
            if (swerveConsole != null)
                swerveConsole.write(
                        "Current Heading: " + gyroHeading,
                        "Desired Angle: " + desiredHeading,
                        "Rotation Speed: " + rotationSpeed,
                        "Translation Vector: " + desiredMovement.toString(Vector2D.VectorCoordinates.POLAR),
                        "Magnitude: " + fieldCentricTranslation.magnitude);
        }

        // Robot centric control.
        else if (joystickControlMethod == JoystickControlMethod.ROBOT_CENTRIC)
        {
            // Receive controller input.
            if (opModeSituation == EnhancedOpMode.AutoOrTeleop.TELEOP)
            {
                desiredMovement = HTGamepad.CONTROLLER1.leftJoystick();
                if (desiredMovement.magnitude < .0005)
                    desiredMovement = Vector2D.ZERO;

                rotationSpeed = HTGamepad.CONTROLLER1.gamepad.right_stick_x;
            }

            // Set vector targets for wheels.
            driveVector = desiredMovement.multiply(speedControl.driveSpeed);

            // Write some information to the telemetry console.
            if (swerveConsole != null)
                swerveConsole.write(
                        "Desired Angle: " + desiredHeading,
                        "Rotation Speed: " + rotationSpeed,
                        "Translation Vector: " + desiredMovement.toString(Vector2D.VectorCoordinates.POLAR));
        }

        // By setting vector targets, SwerveModules will take care of their own orientation.
        if (swomniControlMode == SwomniControlMode.SWERVE_DRIVE)
        {
            for (int i = 0; i < swerveModules.length; i++)
                Vector2D.polar(rotationSpeed * speedControl.turnSpeed, WHEEL_ORIENTATIONS[i]).add(driveVector);
        }
        else if (swomniControlMode == SwomniControlMode.HOLONOMIC)
        {
            for (int i = 0; i < swerveModules.length; i++)
                swerveModules[i].setVectorTarget(Vector2D.polar(
                        rotationSpeed * speedControl.turnSpeed + driveVector.magnitude * speedControl.driveSpeed * Math.cos(Math.toDegrees(Math.abs(WHEEL_ORIENTATIONS[i] - driveVector.angle))),
                        WHEEL_ORIENTATIONS[i]));
        }

        updateCanDrive();

        // A bit of latency before update.
        return 100;
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

    /**
     * Turns the robot to some pre-specified heading according to the gyro
     */
    public void turnRobotToHeading(double heading, double precisionRequired, long msMax, Flow flow) throws InterruptedException
    {
        setDesiredHeading(heading);

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < msMax && Math.abs(gyro.getHeading() - heading) > precisionRequired)
        {
            if (swerveUpdatePackage.getUpdateMode() == ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS)
                synchronousUpdate();

            flow.yield();
        }

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
