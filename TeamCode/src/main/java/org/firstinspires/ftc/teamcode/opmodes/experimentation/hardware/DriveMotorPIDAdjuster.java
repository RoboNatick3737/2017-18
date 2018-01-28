package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;

import hankextensions.EnhancedOpMode;

@TeleOp(name="Drive Motor PID Adjuster", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class DriveMotorPIDAdjuster extends EnhancedOpMode
{
    private EncoderMotor frontLeft, backLeft, frontRight, backRight;

    @Override
    protected void onRun() throws InterruptedException
    {
        // All of the drive motors and their respective PID.
        frontLeft = new EncoderMotor(
                "Front Left",
                hardware.initialize(DcMotor.class, "Front Left"),
                new PIDConstants(.0008, 0, 0, 0, 40000000),
                407, 7.62, DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = new EncoderMotor(
                "Front Right",
                hardware.initialize(DcMotor.class, "Front Right"),
                new PIDConstants(.0008, 0, 0, 0, 40000000),
                202, 7.62, DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = new EncoderMotor(
                "Back Left",
                hardware.initialize(DcMotor.class, "Back Left"),
                new PIDConstants(.0008, 0, 0, 0, 40000000),
                202, 7.62, DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = new EncoderMotor(
                "Back Right",
                hardware.initialize(DcMotor.class, "Back Right"),
                new PIDConstants(.0008, 0, 0, 0, 40000000),
                475, 7.62, DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

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

                flow.msPause(30);
            }
        }

        flow.msPause(3000);
    }
}
