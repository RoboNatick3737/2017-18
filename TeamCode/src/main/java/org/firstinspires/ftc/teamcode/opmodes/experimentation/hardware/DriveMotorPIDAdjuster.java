package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;

import hankextensions.EnhancedOpMode;

@TeleOp(name="Drive Motor PID Adjuster", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class DriveMotorPIDAdjuster extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        // All of the drive motors and their respective PID.
        EncoderMotor[] driveMotors = Robot.getDriveMotors(hardware, DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        for (EncoderMotor driveMotor : driveMotors)
            figureOutPIDConstantsFor(driveMotor);
    }

    private void figureOutPIDConstantsFor(EncoderMotor motor) throws InterruptedException
    {
        ProcessConsole swerveConsole = log.newProcessConsole(motor.motorName);

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

        swerveConsole.destroy();

        flow.msPause(3000);
    }
}
