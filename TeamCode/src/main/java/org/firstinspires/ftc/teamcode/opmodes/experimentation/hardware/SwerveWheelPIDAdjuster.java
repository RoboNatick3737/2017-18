package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveModule;

import hankextensions.structs.Vector2D;

import hankextensions.EnhancedOpMode;

@TeleOp(name="Swerve Wheel PID Adjuster", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class SwerveWheelPIDAdjuster extends EnhancedOpMode
{
    private ScheduledTaskPackage taskPackage;

    @Override
    protected void onRun() throws InterruptedException
    {
        taskPackage = new ScheduledTaskPackage(this, "Swerve Wheel Adjustments");

        SwerveModule[] swerveModules = Robot.getSwerveModules(hardware, DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        for (SwerveModule module : swerveModules)
            figureOutPIDConstantsFor(module);
    }

    private void figureOutPIDConstantsFor(SwerveModule swerveModule) throws InterruptedException
    {
        taskPackage.add(swerveModule);
        taskPackage.run();

        Vector2D desiredRotation;

        ProcessConsole swervePIDConsole = log.newProcessConsole(swerveModule.moduleName + " PID");

        while (!gamepad1.start)
        {
            desiredRotation = Vector2D.rectangular(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotateBy(-90);

            if (desiredRotation.magnitude > .05)
                swerveModule.setVectorTarget(desiredRotation);

            // Simulate controller latency of 90 ms
            for (int i = 0; i < 3; i++)
            {
                if (gamepad1.a)
                    swerveModule.pidController.pidConstants.kP += .0001;
                else if (gamepad1.y)
                    swerveModule.pidController.pidConstants.kP -= .0001;

                if (gamepad1.b)
                    swerveModule.pidController.pidConstants.kI += .0001;
                else if (gamepad1.x)
                    swerveModule.pidController.pidConstants.kI -= .0001;

                if (gamepad1.dpad_up)
                    swerveModule.pidController.pidConstants.kD += .00001;
                else if (gamepad1.dpad_down)
                    swerveModule.pidController.pidConstants.kD -= .00001;

                if (gamepad1.dpad_left)
                    swerveModule.pidController.pidConstants.errorThreshold += .01;
                else if (gamepad1.dpad_right)
                    swerveModule.pidController.pidConstants.errorThreshold -= .01;

                swervePIDConsole.write(
                        "kP is " + swerveModule.pidController.pidConstants.kP,
                        "kI is " + swerveModule.pidController.pidConstants.kI,
                        "kD is " + swerveModule.pidController.pidConstants.kD,
                        "error threshold is " + swerveModule.pidController.pidConstants.errorThreshold,
                        "x = " + gamepad1.left_stick_x + " and y = " + gamepad1.left_stick_y
                );

                flow.msPause(30);
            }
        }

        taskPackage.stop();
        taskPackage.remove(swerveModule);
        swerveModule.turnMotor.setPosition(0.5);

        swervePIDConsole.destroy();

        flow.msPause(3000);
    }
}
