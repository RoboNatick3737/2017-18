package org.firstinspires.ftc.teamcode.robot.hardware;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.structs.pid.PIDController;

public class EncoderMotor
{
    private enum MotorPIDControlMethod {MODERN_ROBOTICS, CUSTOM}
    private final MotorPIDControlMethod controlMethod;

    /**
     * The motor reference to which this corresponds.
     */
    public final DcMotor motor;

    /**
     * The PID controller which this motor uses to stabilize itself.
     */
    public final PIDController pidController;

    /**
     * The process console which this motor needs to output data.
     */
    private final ProcessConsole processConsole;

    /**
     * The number of encoder ticks it takes this motor to rotate 360 degrees once.
     */
    private final int ENCODER_TICKS_PER_REVOLUTION;

    /**
     * The wheel circumference which this motor drives (public so that SwerveWheel
     * can look at this to know how much to correct by)
     */
    public final double WHEEL_CIRCUMFERENCE;

    /**
     * For custom PID control.
     */
    public EncoderMotor(String motorName, DcMotor motor, PIDConstants motorPID, int encoderTicksPerWheelRevolution, double wheelDiameterCM)
    {
        controlMethod = MotorPIDControlMethod.CUSTOM;

        this.motor = motor;
        resetEncoder();

        this.pidController = new PIDController(motorPID);

        // The wheel which the motor drives.
        ENCODER_TICKS_PER_REVOLUTION = encoderTicksPerWheelRevolution;
        WHEEL_CIRCUMFERENCE = wheelDiameterCM * Math.PI;

        processConsole = LoggingBase.instance.newProcessConsole(motorName + " Motor Process Console");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * For MR PID control.
     */
    public EncoderMotor(String motorName, DcMotor motor, double wheelDiameterCM)
    {
        controlMethod = MotorPIDControlMethod.MODERN_ROBOTICS;

        this.motor = motor;

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // The wheel which the motor drives.
        WHEEL_CIRCUMFERENCE = wheelDiameterCM * Math.PI;

        ENCODER_TICKS_PER_REVOLUTION = 0; // not used.
        pidController = null; // Also not used.

        processConsole = LoggingBase.instance.newProcessConsole(motorName + " Motor Process Console");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Resets the motor encoder.
     */
    public void resetEncoder()
    {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // The desired velocity of the motor in meters per second.
    private double desiredVelocity = 0;
    private double lastMotorPosition = 0;
    private long lastAdjustmentTime = 0;
    private double currentPower = 0, currentVelocity = 0;

    /**
     * Tells this motor the number of revolutions that it should be moving per second.
     * @param velocity the angular velocity of the motor.
     */
    public void setVelocity(double velocity)
    {
        if (controlMethod == MotorPIDControlMethod.CUSTOM)
        {
            // Some really quick adjustments we can make.
            if (Math.abs(velocity) < .001) {
                motor.setPower(0);
                currentPower = 0;
            } else if (currentPower < 0 && desiredVelocity > 0) {
                motor.setPower(.1);
                currentPower = .1;
            } else if (currentPower > 0 && desiredVelocity < 0) {
                motor.setPower(-.1);
                currentPower = -.1;
            }

            desiredVelocity = velocity;
        }
        else if (controlMethod == MotorPIDControlMethod.MODERN_ROBOTICS)
        {
            motor.setPower(velocity);
        }
    }

    /**
     * Controls updating PID for the motor.
     */
    public void updatePID()
    {
        if (controlMethod != MotorPIDControlMethod.CUSTOM)
            return;

        // Rare
        if (System.nanoTime() - lastAdjustmentTime == 0)
            return;

        if (!pidController.canUpdate())
            return;

        // Calculate PID by finding the number of ticks the motor SHOULD have gone minus the amount it actually went.
        currentVelocity = (((motor.getCurrentPosition() - lastMotorPosition) / ENCODER_TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE) / ((System.nanoTime() - lastAdjustmentTime)) *  1000000000.0; // big # is for seconds to nanoseconds conversion.
        currentPower += pidController.calculatePIDCorrection(desiredVelocity - currentVelocity);
        motor.setPower(Range.clip(currentPower, -1, 1));

        processConsole.write(
                "Current position: " + lastMotorPosition,
                "Desired velocity: " + desiredVelocity + " cm/s",
                "Current velocity: " + currentVelocity + " cm/s",
                "Current power: " + currentPower,
                "PID constants: " + pidController.pidConstants.kP + ", " + pidController.pidConstants.kD);

        lastMotorPosition = motor.getCurrentPosition();
        lastAdjustmentTime = System.nanoTime();
    }
}
