/**
 * This class represents a single swerve motor, one of the four motors which are used to control the swerve drive.  It consists
 * of:
 *  1. A drive motor.  This motor powers the movement of the wheel on the swerve motor.
 *  2. A turning motor (aka a vex motor), which is technically a servo but simply controls the heading of the wheel.
 *  3. The encoder motor.  This is a dirty trick which involves us adding an external encoder to the vex motor, and plugging
 *     the encoder into a motor encoder port.  This works, although it's super gross (maybe we'll find a solution at some point).
 */

package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.sdkextensions.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.sdkextensions.threading.SimpleTask;

public class SwerveMotor
{
    private final String motorName;
    private final EncoderMotor driveMotor;
    private final Servo turnMotor;
    private final EncoderMotor turnMotorPosition;

    public final SwivelTask swivelTask;

    // The vector components which should constitute the direction of this wheel.
    private double mag, theta;

    private final double ENCODER_TICKS_PER_REVOLUTION = 0;

    public SwerveMotor(String motorName, EncoderMotor driveMotor, Servo turnMotor, EncoderMotor turnMotorPosition)
    {
        this.motorName = motorName;
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnMotorPosition = turnMotorPosition;
        this.swivelTask = new SwivelTask();
    }

    /**
     * Since the vex motor requires some time to turn to the correct position (we aren't just using servos, unfortunately), we have
     * to essentially schedule simple tasks to continually update the speed of the vex motor.
     *
     * Since there are four of these, they are placed into a SimpleTaskPackage in the SwerveDrive motor to run them.
     */
    public class SwivelTask extends SimpleTask {
        public SwivelTask() {
            super(motorName + " Turning Task");
        }

        @Override
        protected long onContinueTask() throws InterruptedException
        {
            // Calculate the current degree.
            double currentDegree = turnMotorPosition.motor.getCurrentPosition() % ENCODER_TICKS_PER_REVOLUTION * 360.0;
            if (currentDegree < 0) currentDegree += 360;

            double angleFromDesired = currentDegree - theta;

            // Don't bother with trying to turn if we're pretty much already at the ideal position.
            if (angleFromDesired > 10)
            {
                // If 20 degrees and want to turn to 270, diff for 20 - 270 is -250.  360 - |-250| = 110 < |-250|, so we swap directions.
                if (360 - Math.abs(angleFromDesired) < Math.abs(angleFromDesired))
                    angleFromDesired = Math.signum(angleFromDesired) * (360 - Math.abs(angleFromDesired));

                turnMotor.setPosition(Range.clip(Math.signum(angleFromDesired) * (.6 + .05 * Math.abs(angleFromDesired)), -1, 1));
            }

            // TODO: See whether the division is a good idea, scales power depending on distance from ideal.
            driveMotor.motor.setPower(Range.clip(mag / (Math.abs(angleFromDesired) + 1), -1, 1));

            // The ms before updating again.
            return 10;
        }
    }

    /**
     * Takes the desired rectangular coordinates for this motor, and converts them to polar coordinates.
     */
    private void setVectorTarget(Vector2D target)
    {
        // We could store the vector, but then we'd be re-calculating these values over and over.
        mag = target.magnitude();
        theta = target.angle();
    }
}
