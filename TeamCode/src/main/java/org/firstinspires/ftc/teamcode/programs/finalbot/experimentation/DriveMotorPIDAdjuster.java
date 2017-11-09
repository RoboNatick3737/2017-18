package org.firstinspires.ftc.teamcode.programs.finalbot.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.hardware.pid.PIDConstants;

import hankextensions.Core;
import hankextensions.logging.ProcessConsole;
import hankextensions.threading.Flow;

@TeleOp(name="Drive Motor PID Adjuster", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class DriveMotorPIDAdjuster extends Core
{
    private EncoderMotor frontLeft, backLeft, frontRight, backRight;

    @Override
    protected void INITIALIZE() throws InterruptedException
    {
        // All of the drive motors and their respective PID.
        frontLeft = new EncoderMotor(
                "Front Left",
                initHardwareDevice(DcMotor.class, "Front Left"),
                new PIDConstants(.0008, 0, 0, 0),
                407, 7.62);

        frontRight = new EncoderMotor(
                "Front Right",
                initHardwareDevice(DcMotor.class, "Front Right"),
                new PIDConstants(.0008, 0, 0, 0),
                202, 7.62);

        backLeft = new EncoderMotor(
                "Back Left",
                initHardwareDevice(DcMotor.class, "Back Left"),
                new PIDConstants(.0008, 0, 0, 0),
                202, 7.62);

        backRight = new EncoderMotor(
                "Back Right",
                initHardwareDevice(DcMotor.class, "Back Right"),
                new PIDConstants(.0008, 0, 0, 0),
                475, 7.62);
    }

    @Override
    protected void START() throws InterruptedException
    {
        figureOutPIDConstantsFor(frontLeft);

        figureOutPIDConstantsFor(frontRight);

        figureOutPIDConstantsFor(backLeft);

        figureOutPIDConstantsFor(backRight);
    }

    ProcessConsole swerveConsole;

    private int increment = 0;
    private void figureOutPIDConstantsFor(EncoderMotor motor) throws InterruptedException
    {
        increment++;
        swerveConsole = log.newProcessConsole("PID " + increment);

        while (!gamepad1.start)
        {
            double desiredVelocity = 50 * Range.clip(-gamepad1.left_stick_y , -1, 1);

            motor.setVelocity(desiredVelocity);

            for (int i = 0; i < 3; i++)
            {
                if (gamepad1.a)
                    motor.pidController.pidConstants.kP += .00001;
                else if (gamepad1.y)
                    motor.pidController.pidConstants.kP -= .00001;

                if (gamepad1.b)
                    motor.pidController.pidConstants.kI += .00001;
                else if (gamepad1.x)
                    motor.pidController.pidConstants.kI -= .00001;

                if (gamepad1.dpad_up)
                    motor.pidController.pidConstants.kD += .00001;
                else if (gamepad1.dpad_down)
                    motor.pidController.pidConstants.kD -= .00001;

                if (gamepad1.dpad_left)
                    motor.pidController.pidConstants.errorThreshold += .1;
                else if (gamepad1.dpad_right)
                    motor.pidController.pidConstants.errorThreshold -= .1;

                motor.updatePID();

                swerveConsole.write(
                        "Desired speed is " + desiredVelocity,
                        "kP is " + motor.pidController.pidConstants.kP,
                        "kI is " + motor.pidController.pidConstants.kI,
                        "kD is " + motor.pidController.pidConstants.kD,
                        "error threshold is " + motor.pidController.pidConstants.errorThreshold
                );

                Flow.msPause(30);
            }
        }

        Flow.msPause(3000);
    }
}
