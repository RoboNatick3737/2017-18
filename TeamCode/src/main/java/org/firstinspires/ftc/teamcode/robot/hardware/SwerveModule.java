package org.firstinspires.ftc.teamcode.robot.hardware;

import android.support.annotation.NonNull;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTask;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.structs.Function;
import org.firstinspires.ftc.teamcode.structs.PIDController;

import hankextensions.structs.Vector2D;

/**
 * Since the vex motor requires some time to turn to the correct position (we aren't
 * just using servos, unfortunately), we have to essentially schedule simple tasks to
 * continually update the speed of the vex motor.
 *
 * Since there are four of these, they are placed into a ScheduledTaskPackage in the
 * SwerveDrive motor to run them.
 *
 * Since rotating the vex motor around once causes the wheel to turn 135 degrees, that
 * also has to be corrected for.
 *
 * Deceleration is also a prominent part of this, since we don't want to slow to zero
 * instantly (hurting the axles).
 */

public class SwerveModule extends ScheduledTask
{
    /**
     * The orientation at which it's OK to drive the module.
     */
    private static final double DRIVING_OK_THRESHOLD = 45;

    /**
     * So when the drive motor runs it applies a torque to the swerve module, so this tries to
     * offset.  Sometimes unpredictable so disable/enable at will.
     */
    private static final boolean APPLY_DRIVE_MOTOR_TORQUE_CORRECTION = true;
    public static double TORQUE_CORRECTION_FACTOR = -.0001; // otherwise with chains on outside they point toward center when going forward

    /**
     * I'm not 100% sure but in case the absolute encoder position hasn't changed and the module
     * is definitely turning, immediately request the next update.
     */
    private static final boolean ABSOLUTE_ENCODER_UPDATE_CHECK = false, DAMP_TURN_SPEED_IF_SO = false;
    
    // Swerve wheel specific components.
    private final String moduleName;
    public final EncoderMotor driveMotor;
    private final Servo turnMotor;
    private final AbsoluteEncoder swerveEncoder;
    private final double physicalEncoderOffset;
    
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
    private Vector2D targetVector = Vector2D.polar(0, 0);

    // Swiveling properties.
    private double currentSwivelOrientation = 0;
    public double getCurrentSwivelOrientation()
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
    // Set driving state.
    public void setDrivingState(boolean drivingEnabled)
    {
        this.drivingEnabled = drivingEnabled;

        if (!this.drivingEnabled)
            driveMotor.motor.setPower(0);
    }

    // Required for absolute encoder position verification
    private int numAbsoluteEncoderSkips = 0;

    // Finally, the PID controller components which prevents wheel oscillation.
    public final Function errorResponder;
    private long updateRateMS;

    /**
     * Instantiates the SwerveModule with the data it requires.
     * @param moduleName  The module name (will appear with this name in logging).
     * @param driveMotor  The drive motor for the module.
     * @param turnMotor   The turning vex motor for the module.
     * @param swerveEncoder  The absolute encoder on the vex motor.
     * @param pidController   The PID constants for aligning the vex motor.
     * @param physicalEncoderOffset  The degree offset of the absolute encoder from zero.
     */
    public SwerveModule(
            String moduleName,
            EncoderMotor driveMotor,
            Servo turnMotor,
            AbsoluteEncoder swerveEncoder,
            PIDController pidController,
            double physicalEncoderOffset)
    {
        this(moduleName, driveMotor, turnMotor, swerveEncoder, pidController, (long)(pidController.minimumNanosecondGap / 1e3), physicalEncoderOffset);
    }
    
    /**
     * Instantiates the SwerveModule with the data it requires.
     * @param moduleName  The module name (will appear with this name in logging).
     * @param driveMotor  The drive motor for the module.
     * @param turnMotor   The turning vex motor for the module.
     * @param swerveEncoder  The absolute encoder on the vex motor.
     * @param errorResponder   The error responder for aligning the vex motor.
     * @param physicalEncoderOffset  The degree offset of the absolute encoder from zero.
     */
    public SwerveModule(
            String moduleName,
            EncoderMotor driveMotor,
            Servo turnMotor,
            AbsoluteEncoder swerveEncoder,
            Function errorResponder,
            long updateRateMS,
            double physicalEncoderOffset)
    {
        this.moduleName = moduleName;
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnMotor.setPosition(0.5);
        this.swerveEncoder = swerveEncoder;
        this.physicalEncoderOffset = physicalEncoderOffset;

        this.errorResponder = errorResponder;
        this.updateRateMS = updateRateMS;
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
    public long onContinueTask() throws InterruptedException
    {
        // If we aren't going to be driving anywhere, don't try to align.
        if (targetVector.magnitude < .00001)
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
                double newSwivelOrientation = swerveEncoder.position();

                // If we're turning but the absolute encoder hasn't registered the turn position change (latency).
                if (Math.abs(currentTurnSpeed) > .15 && Math.abs(newSwivelOrientation - currentSwivelOrientation) < .5)
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
                    return 0;
                }

                currentSwivelOrientation = newSwivelOrientation;
            }
            else
                // Update position.
                currentSwivelOrientation = swerveEncoder.position();

            // Shortest angle from current heading to desired heading.
            double desiredAngle = targetVector.angle;
            double currentAngle = currentSwivelOrientation - physicalEncoderOffset;
            double angleFromDesired = (Vector2D.clampAngle(desiredAngle - currentAngle) + 180) % 360 - 180;
            angleFromDesired = angleFromDesired < -180 ? angleFromDesired + 360 : angleFromDesired;

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

            // Otherwise wait until we calculate drive power.
            if (!APPLY_DRIVE_MOTOR_TORQUE_CORRECTION)
                turnMotor.setPosition(Range.clip(turnPower, 0, 1));

            // For turn motor correction
            double drivePower = 0;

            // Set drive power (if angle between this and desired angle is greater than 90, reverse motor).
            if (drivingEnabled)
            {
                // Scale up/down motor power depending on how far we are from the ideal heading.
                drivePower = targetVector.magnitude / (5 * Math.abs(currentTurnSpeed) + 1);
                if (Math.abs(angleFromDesired) > 90) // Angle to turn != angle desired
                    drivePower *= -1;
                driveMotor.setVelocity(drivePower);
            }

            if (APPLY_DRIVE_MOTOR_TORQUE_CORRECTION)
                // Try to counteract the torque applied by the driving motor.
                turnMotor.setPosition(Range.clip(turnPower + TORQUE_CORRECTION_FACTOR * drivePower, 0, 1));

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
        return updateRateMS;
    }
}
