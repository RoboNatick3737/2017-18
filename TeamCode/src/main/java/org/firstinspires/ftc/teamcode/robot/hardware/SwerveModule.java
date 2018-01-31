/**
 * This class represents a single swerve motor, one of the four motors which are used to control the swerve drive.  It consists
 * of:
 *  1. A drive motor.  This motor powers the movement of the wheel on the swerve motor.
 *  2. A turning motor (aka a vex motor), which is technically a servo but simply controls the heading of the wheel.
 *  3. The encoder motor.  This is a dirty trick which involves us adding an external encoder to the vex motor, and plugging
 *     the encoder into a motor encoder port.  This works, although it's super gross (maybe we'll find a solution at some point).
 */

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
    // Swerve wheel constants.
    private static final double DRIVING_OK_THRESHOLD = 45;

    // Swerve wheel specific components.
    public final String moduleName;
    public final EncoderMotor driveMotor;
    public final Servo turnMotor;
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

    private int updates = 0;

    // The vector components which should constitute the direction and power of this wheel.
    private Vector2D targetVector = Vector2D.polar(0, 0);

    // The boolean which indicates to the parent swerve drive whether this wheel has swiveled to the correct position.
    private boolean swivelAcceptable = true;
    private boolean drivingEnabled = false;

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
        updates++;

        // If we aren't going to be driving anywhere, don't try to align.
        if (targetVector.magnitude < .00001)
        {
            turnMotor.setPosition(0.5);

            if (errorResponder instanceof PIDController)
                ((PIDController) errorResponder).resetController();

            if (driveMotor != null)
                driveMotor.setVelocity(0);

            if (wheelConsole != null)
                // Add console information.
                wheelConsole.write(
                        "Updates: " + updates);
        }
        else
        {
            // Shortest angle from current heading to desired heading.
            double desiredAngle = targetVector.angle;
            double currentAngle = swerveEncoder.position() - physicalEncoderOffset;
            double angleFromDesired = (Vector2D.clampAngle(desiredAngle - currentAngle) + 180) % 360 - 180;
            angleFromDesired = angleFromDesired < -180 ? angleFromDesired + 360 : angleFromDesired;

            // Clip this angle to 90 degree maximum turns.
            double angleToTurn = 0;
            if (angleFromDesired > 90)
                angleToTurn = -angleFromDesired + 180;
            else if (angleFromDesired < -90)
                angleToTurn = -angleFromDesired - 180;
            else
                angleToTurn = angleFromDesired;

            // Set turn power.
            double turnPower = 0.5;

            // Use PID to calculate the correction factor (error bars contained within PID).
            double turnCorrectionFactor = errorResponder.value(angleToTurn);

            // Change the turn factor depending on our distance from the angle desired (180 vs 0)
            if (angleFromDesired > 90 || angleFromDesired < -90)
                turnPower -= turnCorrectionFactor;
            else
                turnPower += turnCorrectionFactor;
            turnMotor.setPosition(Range.clip(turnPower, 0, 1));

            // Set swivel acceptable.
            swivelAcceptable = Math.abs(angleToTurn) < DRIVING_OK_THRESHOLD;

            // Set drive power (if angle between this and desired angle is greater than 90, reverse motor).
            if (drivingEnabled)
            {
                // Scale up/down motor power depending on how far we are from the ideal heading.
                double drivePower = targetVector.magnitude / (5 * Math.abs(turnCorrectionFactor) + 1);

                if (Math.abs(angleFromDesired) > 90) // Angle to turn != angle desired
                    drivePower *= -1;

                driveMotor.setVelocity(drivePower);
            }

            if (wheelConsole != null)
                // Add console information.
                wheelConsole.write(
                        "Vector target: " + targetVector.toString(Vector2D.VectorCoordinates.POLAR),
                        "Current vector: " + targetVector.toString(Vector2D.VectorCoordinates.POLAR),
                        "Angle to turn: " + angleToTurn,
                        "Driving: " + drivingEnabled,
                        "Updates: " + updates);
        }

        // The ms to wait before updating again.
        return updateRateMS;
    }

    /**
     * @return Whether or not the swivel is within the DRIVING_OK_THRESHOLD bounds.
     */
    public boolean atAcceptableSwivelOrientation()
    {
        return swivelAcceptable;
    }

    /**
     * Tells us whether or not we can start driving (all SwerveModules must be within acceptable
     * bounds).
     * @param state  True = driving is ok, false = driving is bad.
     */
    public void setDrivingState(boolean state)
    {
        drivingEnabled = state;

        if (!drivingEnabled)
            driveMotor.motor.setPower(0);
    }
}
