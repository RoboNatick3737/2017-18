/**
 * This class represents a single swerve motor, one of the four motors which are used to control the swerve drive.  It consists
 * of:
 *  1. A drive motor.  This motor powers the movement of the wheel on the swerve motor.
 *  2. A turning motor (aka a vex motor), which is technically a servo but simply controls the heading of the wheel.
 *  3. The encoder motor.  This is a dirty trick which involves us adding an external encoder to the vex motor, and plugging
 *     the encoder into a motor encoder port.  This works, although it's super gross (maybe we'll find a solution at some point).
 */

package org.firstinspires.ftc.teamcode.programs.finalbot.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.hardware.pid.PIDController;
import org.firstinspires.ftc.teamcode.structs.Vector2D;

import org.firstinspires.ftc.teamcode.hardware.EncoderMotor;

import hankextensions.logging.Log;
import hankextensions.logging.ProcessConsole;
import hankextensions.threading.SimpleTask;

public class SwerveWheel
{
    // Swerve wheel constants.
    private static final double NO_MORE_ADJUSTMENTS_THRESHOLD = 2.5;
    private static final double ACCEPTABLE_ORIENTATION_THRESHOLD = 13;

    // Swerve wheel specific components.
    private final String motorName;
    private final EncoderMotor driveMotor;
    private final Servo turnMotor;
    private final AbsoluteEncoder swerveEncoder;
    private final double physicalEncoderOffset;
    public final SwivelTask swivelTask;
    private final ProcessConsole wheelConsole;

    // The vector components which should constitute the direction and power of this wheel.
    private Vector2D targetVector = Vector2D.ZERO;

    // The boolean which indicates to the parent swerve drive whether this wheel has swiveled to the correct position.
    private boolean swivelAcceptable = true;
    private boolean drivingEnabled = false;

    // Finally, the PID controller components which prevents wheel oscillation.
    private final PIDController pidController;

    public SwerveWheel(
            String motorName,
            EncoderMotor driveMotor,
            Servo turnMotor,
            AbsoluteEncoder swerveEncoder,
            PIDConstants pidConstants)
    {
        this(motorName, driveMotor, turnMotor, swerveEncoder, pidConstants, 0);
    }
    public SwerveWheel(
            String motorName,
            EncoderMotor driveMotor,
            Servo turnMotor,
            AbsoluteEncoder swerveEncoder,
            PIDConstants pidConstants,
            double physicalEncoderOffset)
    {
        this.motorName = motorName;
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.swerveEncoder = swerveEncoder;
        this.physicalEncoderOffset = physicalEncoderOffset;

        wheelConsole = Log.instance.newProcessConsole(motorName + " Swivel Console");

        this.pidController = new PIDController(pidConstants);

        this.swivelTask = new SwivelTask();
    }

    /**
     * Takes the desired rectangular coordinates for this motor, and converts them to polar
     * coordinates.
     */
    public void setVectorTarget(Vector2D target)
    {
        targetVector = target;
    }

    /**
     * Since the vex motor requires some time to turn to the correct position (we aren't
     * just using servos, unfortunately), we have to essentially schedule simple tasks to
     * continually update the speed of the vex motor.
     *
     * Since there are four of these, they are placed into a SimpleTaskPackage in the
     * SwerveDrive motor to run them.
     */
    public class SwivelTask extends SimpleTask
    {
        public SwivelTask() {
            super(motorName + " Turning Task");
        }

        // Prevent boxing/unboxing slowdown.
        Vector2D localTargetVector = Vector2D.ZERO, currentVector = Vector2D.ZERO;
        double turnPower, angleFromDesired, angleToTurn, turnCorrectionFactor;

        /**
         * Right here, we're given a vector which we have to match this wheel to as quickly as
         * possible.
         */
        @Override
        protected long onContinueTask() throws InterruptedException
        {
            if (Math.abs(targetVector.magnitude) > 0.000001 && Math.abs(localTargetVector.angle) > 0.000001)
            {
                localTargetVector = Vector2D.clone(targetVector);
            }
            else
            {
                // Don't turn if set to (0, 0), but do stop moving.
                localTargetVector = Vector2D.polar(0, currentVector.angle);
            }

            // Calculate the current degree including the offset.
            currentVector = Vector2D.polar(1, swerveEncoder.position() - physicalEncoderOffset);

            // Shortest angle from current heading to desired heading.
            angleFromDesired = currentVector.leastAngleTo(localTargetVector);

            // Clip this angle to 90 degree maximum turns.
            if (angleFromDesired > 90)
                angleToTurn = -angleFromDesired + 180;
            else if (angleFromDesired < -90)
                angleToTurn = -angleFromDesired - 180;
            else
                angleToTurn = angleFromDesired;

            // Set turn power.
            turnPower = 0.5;
            if (Math.abs(angleToTurn) > NO_MORE_ADJUSTMENTS_THRESHOLD)
            {
                turnCorrectionFactor = pidController.calculatePIDPower(angleToTurn);//Math.signum(angleToTurn) * (.0006 * Math.pow(Math.abs(angleToTurn), 2));

                // Change the turn factor depending on our distance from the angle desired (180 vs 0)
                if (angleFromDesired > 90 || angleFromDesired < -90)
                    turnPower -= turnCorrectionFactor;
                else
                    turnPower += turnCorrectionFactor;
            }
            turnMotor.setPosition(Range.clip(turnPower, 0, 1));

            // Set swivel acceptable.
            swivelAcceptable = Math.abs(angleToTurn) < ACCEPTABLE_ORIENTATION_THRESHOLD;

            // Set drive power (if angle between this and desired angle is greater than 90, reverse motor).
            if (drivingEnabled)
            {
                if (Math.abs(angleFromDesired) > 90) // Angle to turn != angle desired
                    driveMotor.motor.setPower(-localTargetVector.magnitude);
                else
                    driveMotor.motor.setPower(localTargetVector.magnitude);
            }
            else
            {
                driveMotor.motor.setPower(0);
            }

            // Add console information.
            wheelConsole.write(
                    "Vector target: " + localTargetVector.toString(Vector2D.VectorCoordinates.POLAR),
                    "Current vector: " + currentVector.toString(Vector2D.VectorCoordinates.POLAR),
                    "Angle from desired: " + angleFromDesired,
                    "Angle to turn: " + angleToTurn);

            // The ms to wait before updating again.
            return 10;
        }
    }

    public boolean atAcceptableSwivelOrientation()
    {
        return swivelAcceptable;
    }

    public void setDrivingState(boolean state)
    {
        drivingEnabled = state;
    }
}
