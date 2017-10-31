package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class EncoderMotor
{
    public final DcMotor motor;
    public EncoderMotor(DcMotor motor)
    {
        this(motor, DcMotorSimple.Direction.FORWARD);
    }
    public EncoderMotor(DcMotor motor, DcMotorSimple.Direction direction)
    {
        this.motor = motor;
        motor.setDirection(direction);
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
