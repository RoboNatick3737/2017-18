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

import org.firstinspires.ftc.teamcode.structs.Vector2D;

import org.firstinspires.ftc.teamcode.hardware.EncoderMotor;

import hankextensions.logging.Log;
import hankextensions.logging.ProcessConsole;
import hankextensions.threading.SimpleTask;

public class SwerveWheel
{
    // Swerve wheel constants.
    private static final double NO_MORE_ADJUSTMENTS_THRESHOLD = 2.5;
    private static final double ACCEPTABLE_ORIENTATION_THRESHOLD = 10;

    // Swerve wheel specific components.
    private final String motorName;
    private final EncoderMotor driveMotor;
    private final Servo turnMotor;
    private final AbsoluteEncoder swerveEncoder;
    private final double physicalEncoderOffset;

    public final SwivelTask swivelTask;
    private final ProcessConsole wheelConsole;

    // The vector components which should constitute the direction and power of this wheel.
    private double mag = 0, theta = 0;
    private boolean drivingEnabled = false;

    // The boolean which indicates to the parent swerve drive whether this wheel has swiveled to the correct position.
    private boolean swivelAcceptable = true;

    public SwerveWheel(String motorName, EncoderMotor driveMotor, Servo turnMotor, AbsoluteEncoder swerveEncoder) {
        this(motorName, driveMotor, turnMotor, swerveEncoder, 0);
    }
    public SwerveWheel(String motorName, EncoderMotor driveMotor, Servo turnMotor, AbsoluteEncoder swerveEncoder, double physicalEncoderOffset)
    {
        this.motorName = motorName;
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.swerveEncoder = swerveEncoder;
        this.physicalEncoderOffset = physicalEncoderOffset;

        wheelConsole = Log.instance.newProcessConsole(motorName + " Swivel Console");

        this.swivelTask = new SwivelTask();
    }

    /**
     * Takes the desired rectangular coordinates for this motor, and converts them to polar coordinates.
     */
    public void setVectorTarget(Vector2D target)
    {
        // We could store the vector, but then we'd be re-calculating these values over and over.
        mag = target.magnitude;
        theta = target.angle;
        if (theta >= 180) {
            theta -= 180;
            mag *= -1;
        }
    }

    /**
     * Since the vex motor requires some time to turn to the correct position (we aren't just using servos, unfortunately), we have
     * to essentially schedule simple tasks to continually update the speed of the vex motor.
     *
     * Since there are four of these, they are placed into a SimpleTaskPackage in the SwerveDrive motor to run them.
     */
    public class SwivelTask extends SimpleTask
    {
        public SwivelTask() {
            super(motorName + " Turning Task");
        }

        // Prevent boxing/unboxing slowdown.
        double currentDegree, turnPower, angleFromDesired;

        @Override
        protected long onContinueTask() throws InterruptedException
        {
            // Initially at stopped position (gets changed later)
            turnPower = 0.5;

            // Calculate the current degree including the offset.
            currentDegree = Vector2D.clampAngle(swerveEncoder.position() - physicalEncoderOffset);

            // Wrap encoder degree to [0,180]
            if (currentDegree >= 180)
                currentDegree -= 180;

            // Figure out whether it would be easier to turn backwards to theta as opposed to forwards.  If so, reverse the power.
            if (Math.abs(theta - (currentDegree > 90 ? currentDegree - 180 : currentDegree + 180)) < Math.abs(theta - currentDegree))
                angleFromDesired = theta - (currentDegree > 90 ? currentDegree - 180 : currentDegree + 180);
            else
                angleFromDesired = theta - currentDegree;

            // Adjust if we're significantly away from the vector threshold.
            if (Math.abs(angleFromDesired) > NO_MORE_ADJUSTMENTS_THRESHOLD)
                turnPower += Math.signum(angleFromDesired) * (.0009 * Math.pow(angleFromDesired , 2));

            // Turn vex motor.
            turnMotor.setPosition(turnPower);

            // Sometimes it's best just to orient the wheel.
            swivelAcceptable = Math.abs(angleFromDesired) < ACCEPTABLE_ORIENTATION_THRESHOLD;

            // Only drive when the swerve drive parent says it's fine to do so.
            if (drivingEnabled)
                driveMotor.motor.setPower(Range.clip(mag, -1, 1));
            else
                driveMotor.motor.setPower(0);

            // Add console information.
            wheelConsole.write(
                    "Current degree is " + currentDegree,
                    "Angle from desired is " + angleFromDesired,
                    "Desired theta is " + theta);

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
