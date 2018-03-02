package org.firstinspires.ftc.teamcode.robot.hardware;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.util.Range;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.Flow;
import dude.makiah.androidlib.threading.ScheduledTask;
import dude.makiah.androidlib.threading.ScheduledTaskPackage;

import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.input.HTGamepad;

import hankutanku.math.Function;
import hankutanku.math.ModifiedPIDController;
import hankutanku.math.PIDController;
import hankutanku.math.ParametrizedVector;
import hankutanku.math.SingleParameterRunnable;

import hankutanku.phonesensors.Gyro;
import hankutanku.math.Vector2D;

/**
 * The SwomniDrive contains 4 SwomniModule instances to which a number of vectors are specified
 * in order to determine the direction in which movement will occur.
 */
public class SwomniDrive extends ScheduledTask
{
    private static final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH)); // Will be 45 degrees with perfect square dimensions.
    private static final double[] WHEEL_ORIENTATIONS = {ROBOT_PHI - 90, (180 - ROBOT_PHI) - 90, (180 + ROBOT_PHI) - 90, (360 - ROBOT_PHI) - 90};
    private static final Function FIELD_CENTRIC_TURN_CONTROLLER = new ModifiedPIDController(.0081, .001, 0, 5, new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 40), -1000, 1000, .95);
    private static TimeMeasure controlUpdateLatency = new TimeMeasure(TimeMeasure.Units.MILLISECONDS, 100);

    // Total vector displacement from desired movement combinations, may be off from current but allows driving inaccuracies to be corrected in later drives.
    private Vector2D cumulativeRobotFieldPosition = Vector2D.ZERO;

    // Robot reference (for gyro and such).
    private final Gyro gyro;
    private final EnhancedOpMode.AutoOrTeleop opModeSituation;

    // The SwomniModule instances which constitute the swerve drive: frontLeft, backLeft, backRight, frontRight respectively.
    public final SwomniModule[] swomniModules;

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
    public SwomniDrive(EnhancedOpMode.AutoOrTeleop opModeSituation, Gyro gyro, SwomniModule[] modules)
    {
        this.opModeSituation = opModeSituation;
        this.gyro = gyro;

        // The swerve wheels.
        this.swomniModules = modules;

        if (opModeSituation == EnhancedOpMode.AutoOrTeleop.AUTONOMOUS)
        {
            // Initialize the task package regardless we need it atm, better to have it and skip the initialization sequence.
            swerveUpdatePackage = new ScheduledTaskPackage(EnhancedOpMode.instance, "Swerve Turn Alignments",
                    this, this.swomniModules[0], this.swomniModules[1], this.swomniModules[2], this.swomniModules[3],
                    this.swomniModules[0].driveMotor, this.swomniModules[1].driveMotor, this.swomniModules[2].driveMotor, this.swomniModules[3].driveMotor);
        }

        // Teleop mode is weird... we don't use drive PID.
        else
        {
            // Initialize the task package regardless we need it atm, better to have it and skip the initialization sequence.
            swerveUpdatePackage = new ScheduledTaskPackage(EnhancedOpMode.instance, "Swerve Turn Alignments",
                    this, this.swomniModules[0], this.swomniModules[1], this.swomniModules[2], this.swomniModules[3]);

            for (SwomniModule module : modules)
                module.setEnableDrivePID(false);
        }

        setSwomniControlMode(SwomniControlMode.SWERVE_DRIVE);

        swerveConsole = LoggingBase.instance.newProcessConsole("Swerve Console");

        stop();
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

        if (swomniControlMode == SwomniControlMode.SWERVE_DRIVE)
        {
            for (SwomniModule swomniModule : swomniModules)
            {
                swomniModule.setDriveMotorTorqueCorrectionEnabled(true);
                swomniModule.setEnablePassiveAlignmentCorrection(false);
            }
        }
        else
        {
            for (SwomniModule swomniModule : swomniModules)
            {
                swomniModule.setDriveMotorTorqueCorrectionEnabled(false);
                swomniModule.setEnablePassiveAlignmentCorrection(true);
            }
        }
    }
    public SwomniControlMode getSwomniControlMode()
    {
        return swomniControlMode;
    }
    // endregion

    private void updateCanDrive()
    {
        // Updates the swerve modules on whether we can drive.
        boolean drivingCanStart = true;
        for (SwomniModule wheel : swomniModules)
        {
            if (!wheel.atAcceptableSwivelOrientation())
            {
                drivingCanStart = false;
                break;
            }
        }
        for (SwomniModule wheel : swomniModules)
            wheel.setDrivingState(drivingCanStart);
    }

    /**
     * Runs every update, just does a lot of vector algebra depending on the drive mode we're in.
     */
    @Override
    protected TimeMeasure onContinueTask() throws InterruptedException
    {
        // Simple code for tank drive.
        if (swomniControlMode == SwomniControlMode.TANK_DRIVE)
        {
            if (opModeSituation == EnhancedOpMode.AutoOrTeleop.TELEOP)
            {
                double rotationSpeed = HTGamepad.CONTROLLER1.leftJoystick().x - HTGamepad.CONTROLLER1.rightJoystick().x;
                double driveSpeed = (HTGamepad.CONTROLLER1.leftJoystick().x + HTGamepad.CONTROLLER1.rightJoystick().x) / 2.0;

                for (int i = 0; i < swomniModules.length; i++)
                    swomniModules[i].setVectorTarget(
                            Vector2D.polar(0.25 * rotationSpeed * speedControl.turnSpeed, WHEEL_ORIENTATIONS[i])
                                    .add(Vector2D.polar(driveSpeed * speedControl.driveSpeed, 0)));
            }

            updateCanDrive();

            return controlUpdateLatency;
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
                        return controlUpdateLatency;
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
            Vector2D fieldCentricTranslation = desiredMovement.rotateBy(-desiredHeading);

            // Don't bother trying to be more accurate than 8 degrees while turning.
            rotationSpeed = FIELD_CENTRIC_TURN_CONTROLLER.value(-angleOff);

            if (opModeSituation == EnhancedOpMode.AutoOrTeleop.AUTONOMOUS)
                if (Math.abs(rotationSpeed) > .324)
                    rotationSpeed = Math.signum(rotationSpeed) * .324;

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

                rotationSpeed = -HTGamepad.CONTROLLER1.rightJoystick().y;
            }

            // Set vector targets for wheels.
            driveVector = desiredMovement.multiply(speedControl.driveSpeed);

            // Write some information to the telemetry console.
            if (swerveConsole != null)
                swerveConsole.write(
                        "Desired Angle: " + desiredHeading,
                        "Rotation Speed: " + rotationSpeed,
                        "Translation Vector: " + desiredMovement.toString(Vector2D.VectorCoordinates.POLAR),
                        FIELD_CENTRIC_TURN_CONTROLLER instanceof PIDController ? "PID: " + ((PIDController)FIELD_CENTRIC_TURN_CONTROLLER).summary() : "");
        }

        // By setting vector targets, SwerveModules will take care of their own orientation.
        if (swomniControlMode == SwomniControlMode.SWERVE_DRIVE)
        {
            for (int i = 0; i < swomniModules.length; i++)
                swomniModules[i].setVectorTarget(
                        Vector2D.polar(rotationSpeed * speedControl.turnSpeed, WHEEL_ORIENTATIONS[i])
                                .add(driveVector));
        }
        else if (swomniControlMode == SwomniControlMode.HOLONOMIC)
        {
            for (int i = 0; i < swomniModules.length; i++)
            {
                double angleOff = (Vector2D.clampAngle(WHEEL_ORIENTATIONS[i] - driveVector.angle) + 180) % 360 - 180;
                angleOff = angleOff < -180 ? angleOff + 360 : angleOff;

                swomniModules[i].setVectorTarget(
                        Vector2D.polar(
                                rotationSpeed * speedControl.turnSpeed + 2 * driveVector.magnitude * Math.cos(Math.toRadians(angleOff)),
                                WHEEL_ORIENTATIONS[i]));
            }
        }

        updateCanDrive();

        // A bit of latency before update.
        return controlUpdateLatency;
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
     * Drives a certain distance with a variable heading/power.
     * @param movement  Represents the plotted movement over time (just current time is plugged
     *                   into this vector parametrization to decide where we should be).
     * @param totalTime The time that this drive should take (will take longer if it needs longer).
     * @param lookahead The time to look ahead for driving.
     * @param runnable  Optional runnable to run every loop.
     * @param flow      Tells it how to pause itself safely.
     */
    public void purePursuit(ParametrizedVector movement, TimeMeasure totalTime, TimeMeasure lookahead, double endpointLeniency, SingleParameterRunnable runnable, Flow flow) throws InterruptedException
    {
        // The timestamp when we start.
        final long startDriveTime = System.currentTimeMillis();

        // The fraction of the totalTime which is the lookahead.
        final double lookaheadFactor = lookahead.durationIn(TimeMeasure.Units.MILLISECONDS) / totalTime.durationIn(TimeMeasure.Units.MILLISECONDS);

        // The end point of the movement.
        final Vector2D movementEndpoint = movement.getVector(1);

        while (true)
        {
            // Current displacement vector.
            Vector2D currentPositionOnField = Vector2D.average(
                    swomniModules[0].getDisplacementVector(),
                    swomniModules[1].getDisplacementVector(),
                    swomniModules[2].getDisplacementVector(),
                    swomniModules[3].getDisplacementVector());

            // Total time spent driving so far.
            long currentMSProgression = System.currentTimeMillis() - startDriveTime;

            // From 0 to 1 to represent how far along we are.
            double driveCompletion = currentMSProgression / totalTime.durationIn(TimeMeasure.Units.MILLISECONDS);

            // Also consider lookahead, but wrap to 0 and 1.
            double movementParameter = Range.clip(driveCompletion + lookaheadFactor, 0, 1);

            // How to move based on the lookahead.
            Vector2D desiredPositionOnField = movement.getVector(movementParameter).add(cumulativeRobotFieldPosition);

            // Movement vector taken by dividing by some constant which represents a conversion from distance to power.
            Vector2D resultingTranslationVector = desiredPositionOnField.subtract(currentPositionOnField).divide(5);

            // Clip to max of 1 speed.
            if (resultingTranslationVector.magnitude > 1)
                resultingTranslationVector = Vector2D.polar(1, resultingTranslationVector.angle);

            // Apply and drive.
            setDesiredMovement(resultingTranslationVector);
            if (swerveUpdatePackage.getUpdateMode() == ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS)
                synchronousUpdate();

            // Any other tasks we should be doing, with a general idea of how far along we are.
            if (runnable != null)
                runnable.run(driveCompletion);

            // Decide whether we can exit based on our current displacement from the end point.
            if (currentPositionOnField.subtract(movementEndpoint).magnitude < endpointLeniency)
                break;

            flow.yield();
        }

        stop();

        // Allows any errors that may have occurred during this movement be fixed in following movements.
        cumulativeRobotFieldPosition.add(movementEndpoint);
    }

    /**
     * Drives a certain time with a variable heading/power
     * @param movement The direction to drive
     * @param driveTime the ms to drive
     * @param runnable runnable to run every loop
     * @param flow When to exit
     */
    public void driveTime(ParametrizedVector movement, TimeMeasure driveTime, SingleParameterRunnable runnable, Flow flow) throws InterruptedException
    {
        long msDrive = driveTime.durationIn(TimeMeasure.Units.MILLISECONDS);

        if (msDrive <= 0)
            return;

        ProcessConsole distanceConsole = LoggingBase.instance.newProcessConsole("Timed Drive");

        long startTime = System.currentTimeMillis();
        long elapsedTime = 0;
        while (elapsedTime < msDrive)
        {
            elapsedTime = System.currentTimeMillis() - startTime;

            setDesiredMovement(movement.getVector(elapsedTime / msDrive));
            distanceConsole.write("Remaining: " + (msDrive - (System.currentTimeMillis() - startTime) + "ms"));
            if (swerveUpdatePackage.getUpdateMode() == ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS)
                synchronousUpdate();

            // Shortcut, callers can provide anonymous methods here.
            if (runnable != null)
                runnable.run(elapsedTime / msDrive);

            flow.yield();
        }

        stop();
        distanceConsole.destroy();

        // Allows any errors that may have occurred during this movement be fixed in following movements.
        cumulativeRobotFieldPosition.add(movement.getVector(1));
    }

    /**
     * Orients all SwerveModules to a given vector with some degree of certainty (for auto)
     */
    public void orientSwerveModules(Vector2D orientationVector, double precisionRequired, TimeMeasure timeMax, Flow flow) throws InterruptedException
    {
        for (int i = 0; i < swomniModules.length; i++)
            swomniModules[i].setVectorTarget(orientationVector);

        orientModules(precisionRequired, timeMax, flow);
    }

    /**
     * Orient all swivel modules for rotation.
     */
    public void orientSwerveModulesForRotation(double precisionRequired, TimeMeasure timeMax, Flow flow) throws InterruptedException
    {
        for (int i = 0; i < swomniModules.length; i++)
            swomniModules[i].setVectorTarget(Vector2D.polar(1, WHEEL_ORIENTATIONS[i]));

        orientModules(precisionRequired, timeMax, flow);
    }

    /**
     * Private method which actually takes care of the updating bit.
     *
     * TODO include asynchronous mode
     */
    private void orientModules(double precisionRequired, TimeMeasure timeMax, Flow flow) throws InterruptedException
    {
        // Have to make a separate package for just swiveling.
        ScheduledTaskPackage updatePackage = new ScheduledTaskPackage(
                flow.parent,
                "Orienting",
                swomniModules[0], swomniModules[1], swomniModules[2], swomniModules[3]);
        updatePackage.setUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < timeMax.durationIn(TimeMeasure.Units.MILLISECONDS))
        {
            // Otherwise update
            updatePackage.synchronousUpdate();

            // Whether orientations need to keep being updated.
            boolean orientationsAreGood = true;
            for (SwomniModule module : swomniModules)
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
    public void turnRobotToHeading(double heading, double precisionRequired, TimeMeasure timeMax, Flow flow) throws InterruptedException
    {
        if (FIELD_CENTRIC_TURN_CONTROLLER instanceof PIDController)
            ((PIDController) FIELD_CENTRIC_TURN_CONTROLLER).resetController();

        setDesiredHeading(heading);

        long start = System.currentTimeMillis();
        int streak = 0;
        while (System.currentTimeMillis() - start < timeMax.durationIn(TimeMeasure.Units.MILLISECONDS))
        {
            if (swerveUpdatePackage.getUpdateMode() == ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS)
                synchronousUpdate();

            if (Math.abs(gyro.getHeading() - heading) < precisionRequired)
            {
                streak++;

                if (streak > 5)
                    break;
            }
            else
                streak = 0;

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
        for (SwomniModule wheel : swomniModules)
            wheel.stopWheel();

        setDesiredMovement(Vector2D.ZERO);

        if (FIELD_CENTRIC_TURN_CONTROLLER instanceof PIDController)
            ((PIDController) FIELD_CENTRIC_TURN_CONTROLLER).resetController();
    }
}
