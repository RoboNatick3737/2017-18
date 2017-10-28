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

import org.firstinspires.ftc.teamcode.structs.Vector2D;

import org.firstinspires.ftc.teamcode.hardware.EncoderMotor;

import hankextensions.logging.Log;
import hankextensions.logging.ProcessConsole;
import hankextensions.threading.SimpleTask;

public class SwerveWheel
{
    private final String motorName;
    private final EncoderMotor driveMotor;
    private final Servo turnMotor;
    private final AbsoluteEncoder swerveEncoder;
    private final double physicalEncoderOffset;

    public final SwivelTask swivelTask;
    private final ProcessConsole wheelConsole;

    // The vector components which should constitute the direction and power of this wheel.
    private double mag, theta;
    private boolean drivingEnabled = true;

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

        @Override
        protected long onContinueTask() throws InterruptedException
        {
            // Calculate the current degree.
            double currentDegree = swerveEncoder.position() - physicalEncoderOffset;

            // Wrap encoder degree
            if (currentDegree >= 360)
                currentDegree -= 360;
            else if (currentDegree < 0)
                currentDegree += 360;

            // Calculate the smallest angle between this wheel and the desired heading.
            double angleFromDesired = (theta - currentDegree + 180) % 360 - 180;
            angleFromDesired = angleFromDesired < -180 ? angleFromDesired + 360 : angleFromDesired;;

            // Calculate and apply turning powers.
            double turnPower = .5; // Stationary vex motor.
            if (Math.abs(angleFromDesired) > 5) // Change correction factor only if there's a significant distance between our heading and the desired one.
                turnPower += Math.signum(angleFromDesired) * (.0007 * Math.pow(Math.abs(angleFromDesired), 2)); // to the power of 2 so that smaller corrections are made close to the desired heading, and significantly larger ones are made further away.

            turnMotor.setPosition(turnPower);

            // Sometimes it's best just to orient the wheel.
            if (drivingEnabled)
                driveMotor.motor.setPower(mag);

            // Add console information.
            wheelConsole.write("Current degree is " + currentDegree, "Angle from desired is " + angleFromDesired);

            // The ms to wait before updating again.
            return 10;
        }
    }

    /**
     * Takes the desired rectangular coordinates for this motor, and converts them to polar coordinates.
     */
    public void setVectorTarget(Vector2D target) {setVectorTarget(target, false);}
    public void setVectorTarget(Vector2D target, boolean disableMovement)
    {
        // We could store the vector, but then we'd be re-calculating these values over and over.
        mag = target.magnitude();
        theta = target.angle();

        if (disableMovement)
            drivingEnabled = false;
        else
            drivingEnabled = true;
    }
}
