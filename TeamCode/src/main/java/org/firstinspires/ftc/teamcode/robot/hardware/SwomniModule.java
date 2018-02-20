package org.firstinspires.ftc.teamcode.robot.hardware;

import android.support.annotation.NonNull;

import dude.makiah.androidlib.logging.LoggingBase;
import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.ScheduledTask;
import dude.makiah.androidlib.threading.TimeMeasure;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import hankutanku.math.Function;
import hankutanku.math.LimitedUpdateRateFunction;
import hankutanku.math.PIDController;

import hankutanku.math.Angle;
import hankutanku.math.Vector2D;

/**
 * Since the vex motor requires some time to turn to the correct position (we aren't
 * just using servos, unfortunately), we have to essentially schedule simple tasks to
 * continually update the speed of the vex motor.
 *
 * Since there are four of these, they are placed into a ScheduledTaskPackage in the
 * SwomniDrive motor to run them.
 *
 * Since rotating the vex motor around once causes the wheel to turn 135 degrees, that
 * also has to be corrected for.
 *
 * Deceleration is also a prominent part of this, since we don't want to slow to zero
 * instantly (hurting the axles).
 */

public class SwomniModule extends ScheduledTask
{
    /**
     * The orientation at which it's OK to drive the module.
     */
    private static final double DRIVING_OK_THRESHOLD = 45;

    /**
     * I'm not 100% sure but in case the absolute encoder position hasn't changed and the module
     * is definitely turning, immediately request the next update.
     */
    private static final boolean ABSOLUTE_ENCODER_UPDATE_CHECK = false, DAMP_TURN_SPEED_IF_SO = false;

    // The PID controllers for each swerve mode (more sensitive on holonomic mode and tank mode).
    private LimitedUpdateRateFunction<Double> errorResponder;
    public final LimitedUpdateRateFunction<Double> swerveErrorResponder, holonomicErrorResponder, tankErrorResponder;
    private SwomniDrive.SwomniControlMode controlMode;
    public void setControlMode(SwomniDrive.SwomniControlMode controlMode)
    {
        this.controlMode = controlMode;

        switch (controlMode)
        {
            case SWERVE_DRIVE:
                this.errorResponder = swerveErrorResponder;
                setDriveMotorTorqueCorrectionEnabled(true);
                setEnablePassiveAlignmentCorrection(false);
                break;

            case HOLONOMIC:
                this.errorResponder = holonomicErrorResponder;
                setDriveMotorTorqueCorrectionEnabled(false);
                setEnablePassiveAlignmentCorrection(true);
                break;

            case TANK_DRIVE:
                this.errorResponder = tankErrorResponder;
                setDriveMotorTorqueCorrectionEnabled(false);
                setEnablePassiveAlignmentCorrection(true);
                break;
        }
    }

    // Swerve wheel specific components.
    private final String moduleName;
    private final Angle physicalEncoderOffset;
    private boolean driveMotorTorqueCorrectionEnabled = true;
    public void setDriveMotorTorqueCorrectionEnabled(boolean enabled)
    {
        this.driveMotorTorqueCorrectionEnabled = enabled;
    }
    private final double driveMotorTorqueCorrection;
    private boolean enablePassiveAlignmentCorrection = false;
    public void setEnablePassiveAlignmentCorrection(boolean enabled)
    {
        this.enablePassiveAlignmentCorrection = enabled;
    }

    // Queried and set for updates.
    public final EncoderMotor driveMotor;
    private final Servo turnMotor;
    private final AbsoluteEncoder swerveEncoder;
    
    // Whether or not we can log.
    private ProcessConsole wheelConsole = null;
    public void setEnableLogging(boolean enabled)
    {
        boolean currentlyEnabled = wheelConsole != null;

        if (enabled == currentlyEnabled)
            return;

        if (enabled)
            wheelConsole = LoggingBase.instance.newProcessConsole(moduleName + " Swivel Console");
        else
        {
            wheelConsole.destroy();
            wheelConsole = null;
        }
    }
    
    // The vector components which should constitute the direction and power of this wheel.
    private Vector2D targetVector = new Vector2D(0, Angle.ZERO);

    // Swiveling properties.
    private Angle currentSwivelOrientation = Angle.ZERO;
    public Angle getCurrentSwivelOrientation()
    {
        return currentSwivelOrientation;
    }
    private double angleLeftToTurn = 0;
    public double getAngleLeftToTurn()
    {
        return angleLeftToTurn;
    }
    private double currentTurnSpeed = 0;
    public boolean atAcceptableSwivelOrientation()
    {
        return Math.abs(angleLeftToTurn) < DRIVING_OK_THRESHOLD;
    }

    // Driving properties.
    private boolean drivingEnabled = false;
    public void setDrivingState(boolean drivingEnabled)
    {
        this.drivingEnabled = drivingEnabled;

        if (!this.drivingEnabled)
            driveMotor.motor.setPower(0);
    }
    private boolean enableDrivePID = true;
    public void setEnableDrivePID(boolean enableDrivePID)
    {
        this.enableDrivePID = enableDrivePID;
    }

    // Required for absolute encoder position verification
    private int numAbsoluteEncoderSkips = 0;
    
    /**
     * Instantiates the SwomniModule with the data it requires.
     * @param moduleName  The module name (will appear with this name in logging).
     * @param driveMotor  The drive motor for the module.
     * @param turnMotor   The turning vex motor for the module.
     * @param swerveEncoder  The absolute encoder on the vex motor.
     * @param swerveErrorResponder   The error responder for aligning the vex motor.
     * @param physicalEncoderOffset  The degree offset of the absolute encoder from zero.
     */
    public SwomniModule(
            String moduleName,
            EncoderMotor driveMotor,
            Servo turnMotor,
            AbsoluteEncoder swerveEncoder,
            LimitedUpdateRateFunction swerveErrorResponder,
            LimitedUpdateRateFunction holonomicErrorResponder,
            LimitedUpdateRateFunction tankErrorResponder,
            Angle physicalEncoderOffset,
            double driveMotorTorqueCorrection)
    {
        this.moduleName = moduleName;
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnMotor.setPosition(0.5);
        this.swerveEncoder = swerveEncoder;
        this.physicalEncoderOffset = physicalEncoderOffset;

        this.swerveErrorResponder = swerveErrorResponder;
        this.holonomicErrorResponder = holonomicErrorResponder;
        this.tankErrorResponder = tankErrorResponder;
        setControlMode(SwomniDrive.SwomniControlMode.SWERVE_DRIVE);

        this.driveMotorTorqueCorrection = driveMotorTorqueCorrection;
    }

    /**
     * Takes the desired rectangular coordinates for this motor, and converts them to polar
     * coordinates.
     */
    public void setVectorTarget(@NonNull Vector2D target)
    {
        targetVector = target;
    }

    /**
     * Manually stops the wheel (perhaps the task updating stopped).
     */
    public void stopWheel()
    {
        setVectorTarget(Vector2D.ZERO);

        turnMotor.setPosition(0.5);

        if (driveMotor != null)
            driveMotor.setVelocity(0);
    }

    /**
     * Right here, we're given a vector which we have to match this wheel to as quickly as
     * possible.
     */
    @Override
    public TimeMeasure onContinueTask() throws InterruptedException
    {
        // If we aren't going to be driving anywhere, don't try to align.
        if (targetVector.magnitude() < .00001 && !enablePassiveAlignmentCorrection)
        {
            turnMotor.setPosition(0.5);
            currentTurnSpeed = 0;

            if (errorResponder instanceof PIDController)
                ((PIDController) errorResponder).pauseController();

            if (driveMotor != null)
                driveMotor.setVelocity(0);

            if (wheelConsole != null)
                // Add console information.
                wheelConsole.write(
                        "Num skips: " + numAbsoluteEncoderSkips,
                        "Angle to turn: " + angleLeftToTurn,
                        errorResponder instanceof PIDController ? ((PIDController) errorResponder).summary() : "Using constant method");
        }
        else
        {
            // Whether or not we should check whether the current module position is different from the last one.
            if (ABSOLUTE_ENCODER_UPDATE_CHECK)
            {
                // currentSwivelOrientation currently represents old swivel orientation.
                Angle newSwivelOrientation = swerveEncoder.position();

                // If we're turning but the absolute encoder hasn't registered the turn position change (latency).
                if (Math.abs(currentTurnSpeed) > .15 && Math.abs(newSwivelOrientation.shortestPathTo(currentSwivelOrientation, Angle.MeasurementType.DEGREES)) < 5)
                {
                    // Record that this happened
                    numAbsoluteEncoderSkips++;

                    if (DAMP_TURN_SPEED_IF_SO)
                    {
                        // Slightly damp the power of the turning
                        currentTurnSpeed *= .95;
                        turnMotor.setPosition(0.5 + currentTurnSpeed);
                    }

                    // Try to immediately update.
                    return TimeMeasure.IMMEDIATE;
                }

                currentSwivelOrientation = newSwivelOrientation;
            }
            else
                // Update position.
                currentSwivelOrientation = swerveEncoder.position();

            // Shortest angle from current heading to desired heading.
            Angle desiredAngle = targetVector.angle();
            Angle currentAngle = currentSwivelOrientation.subtract(physicalEncoderOffset);
            double angleFromDesired = currentAngle.shortestPathTo(desiredAngle, Angle.MeasurementType.DEGREES);

            // Clip this angle to 90 degree maximum turns.
            if (angleFromDesired > 90)
                angleLeftToTurn = -angleFromDesired + 180;
            else if (angleFromDesired < -90)
                angleLeftToTurn = -angleFromDesired - 180;
            else
                angleLeftToTurn = angleFromDesired;

            // Set turn power.
            double turnPower = 0.5;

            // Use PID to calculate the correction factor (error bars contained within PID).  angle left ^ 1.2
            currentTurnSpeed = errorResponder.value(angleLeftToTurn);

            // Change the turn factor depending on our distance from the angle desired (180 vs 0)
            if (angleFromDesired > 90 || angleFromDesired < -90)
                turnPower -= currentTurnSpeed;
            else
                turnPower += currentTurnSpeed;

            // For turn motor correction
            double drivePower = 0;

            // Set drive power (if angle between this and desired angle is greater than 90, reverse motor).
            if (drivingEnabled)
            {
                // Scale up/down motor power depending on how far we are from the ideal heading.
                drivePower = targetVector.magnitude();
                if (Math.abs(angleFromDesired) > 90) // Angle to turn != angle desired
                    drivePower *= -1;

                if (drivePower < .00001)
                {
                    driveMotor.setVelocity(0);
                    driveMotor.motor.setPower(0);
                }
                else
                {
                    if (enableDrivePID)
                        driveMotor.setVelocity(drivePower);
                    else
                        // Works for some reason, used during teleop.
                        driveMotor.motor.setPower(drivePower / 95.0);
                }
            }

            // Try to counteract the torque applied by the driving motor.
            turnMotor.setPosition(Range.clip(turnPower + (driveMotorTorqueCorrectionEnabled ? driveMotorTorqueCorrection * drivePower : 0), 0, 1));

            if (wheelConsole != null)
                // Add console information.
                wheelConsole.write(
                        "Vector target: " + targetVector.toString(Vector2D.VectorCoordinates.POLAR),
                        "Current vector: " + targetVector.toString(Vector2D.VectorCoordinates.POLAR),
                        "Angle to turn: " + angleLeftToTurn,
                        "Num skips: " + numAbsoluteEncoderSkips,
                        errorResponder instanceof PIDController ? ((PIDController) errorResponder).summary() : "Using constant method");
        }

        // The ms to wait before updating again.
        return errorResponder.getUpdateRate();
    }
}
