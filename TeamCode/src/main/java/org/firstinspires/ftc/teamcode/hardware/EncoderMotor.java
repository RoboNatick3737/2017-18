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
    private final int ENCODER_TICKS_WHEEL_REVOLUTION;

    /**
     * The wheel circumference which this motor drives.
     */
    private final double WHEEL_CIRCUMFERENCE;

    public EncoderMotor(String motorName, DcMotor motor)
    {
        this(motorName, motor, new PIDConstants(0.01, 0, 0, 0), 400, 4);
    }
    public EncoderMotor(String motorName, DcMotor motor, PIDConstants motorPID, int encoderTicksPerWheelRevolution, double wheelDiameterCM)
    {
        this.motor = motor;
        resetEncoder();

        this.pidController = new PIDController(motorPID);

        // The wheel which the motor drives.
        ENCODER_TICKS_WHEEL_REVOLUTION = encoderTicksPerWheelRevolution;
        WHEEL_CIRCUMFERENCE = wheelDiameterCM * Math.PI;

        processConsole = Log.instance.newProcessConsole(motorName + " Motor Process Console");
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
        // Some really quick adjustments we can make.
        if (Math.abs(velocity) < .001)
        {
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

    /**
     * Controls updating PID for the motor.
     */
    public void updatePID()
    {
        // Calculate PID by finding the number of ticks the motor SHOULD have gone minus the amount it actually went.
        currentVelocity = ((motor.getCurrentPosition() - lastMotorPosition) / ENCODER_TICKS_WHEEL_REVOLUTION * WHEEL_CIRCUMFERENCE) /  ((System.currentTimeMillis() - lastAdjustmentTime) / 1000.0);
        currentPower += pidController.calculatePIDCorrection(desiredVelocity - currentVelocity);
        motor.setPower(currentPower);

        processConsole.write(
                "Current position: " + lastMotorPosition,
                "Desired velocity: " + desiredVelocity + " cm/s",
                "Current velocity: " + currentVelocity + " cm/s",
                "Current power: " + currentPower,
                "PID constants: " + pidController.pidConstants.kP + ", " + pidController.pidConstants.kD);

        lastMotorPosition = motor.getCurrentPosition();
        lastAdjustmentTime = System.currentTimeMillis();
    }
}
