package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.hardware.pid.PIDController;

import hankextensions.logging.Log;
import hankextensions.logging.ProcessConsole;

public class EncoderMotor
{
    /**
     * The motor reference to which this corresponds.
     */
    public final DcMotor motor;
    public final PIDController pidController;
    private final ProcessConsole processConsole;

    public EncoderMotor(DcMotor motor)
    {
        this(motor, new PIDConstants(0, 0, 0, 0));
    }
    public EncoderMotor(DcMotor motor, PIDConstants motorPID)
    {
        this.motor = motor;

        this.pidController = new PIDController(motorPID);

        resetEncoder();

        processConsole = Log.instance.newProcessConsole("Motor PC");
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
    private boolean firstErrorCorrection = false;
    private double currentPower = 0, currentVelocity;

    /**
     * Tells this motor the number of revolutions that it should be moving per second.
     * @param velocity the angular velocity of the motor.
     */
    public void setVelocity(double velocity)
    {
        if (Math.abs(velocity) <= .001)
        {
            motor.setPower(0);
            currentPower = 0;
        }

        desiredVelocity = velocity;
        firstErrorCorrection = true;
    }

    /**
     * Controls updating PID for the motor.
     */
    public void updatePID()
    {
        if (!firstErrorCorrection)
        {
            // Calculate PID by finding the number of ticks the motor SHOULD have gone minus the amount it actually went.
            currentVelocity = (motor.getCurrentPosition() - lastMotorPosition) /  (System.currentTimeMillis() - lastAdjustmentTime);
            currentPower += pidController.calculatePIDCorrection(desiredVelocity - currentVelocity);
            motor.setPower(currentPower);
        }

        processConsole.write(
                "Current position is " + lastMotorPosition,
                "Desired velocity = " + (desiredVelocity),
                "Current velocity " + currentVelocity);
        lastMotorPosition = motor.getCurrentPosition();
        lastAdjustmentTime = System.currentTimeMillis();
        firstErrorCorrection = false;
    }
}
