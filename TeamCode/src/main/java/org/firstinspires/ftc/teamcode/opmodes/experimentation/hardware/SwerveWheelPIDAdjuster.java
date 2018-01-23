package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveModule;
import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;

import hankextensions.structs.Vector2D;

import hankextensions.EnhancedOpMode;

@TeleOp(name="Swerve Wheel PID Adjuster", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class SwerveWheelPIDAdjuster extends EnhancedOpMode
{
    private SwerveModule frontLeft, backLeft, frontRight, backRight;
    private ScheduledTaskPackage taskPackage;

    @Override
    protected void onRun() throws InterruptedException {
        taskPackage = new ScheduledTaskPackage(this, "Swerve Wheel Adjustments");

        // All of the SwerveWheels (which align on independent threads)
        frontLeft = new SwerveModule(
                "Front Left",
                null,
                hardware.initialize(Servo.class, "Front Left Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Left Vex Encoder")),
                new PIDConstants(0.013042, 0, 0.000608, 5.194, 40000000),
                61.58);

        frontRight = new SwerveModule(
                "Front Right",
                null,
                hardware.initialize(Servo.class, "Front Right Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Right Vex Encoder")),
                new PIDConstants(0.012465, 0, 0.000945, 2.5, 40000000),
                228.38);

        backLeft = new SwerveModule(
                "Back Left",
                null,
                hardware.initialize(Servo.class, "Back Left Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Left Vex Encoder")),
                new PIDConstants(0.0127, 0, 0.000704, 2.85, 40000000),
                43.636);

        backRight = new SwerveModule(
                "Back Right",
                null,
                hardware.initialize(Servo.class, "Back Right Vex Motor"),
                new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Right Vex Encoder")),
                new PIDConstants(0.01304, 0, 0.000669, 5.678, 40000000),
                257.24);

        waitForStart();

        figureOutPIDConstantsFor(frontLeft);

        figureOutPIDConstantsFor(frontRight);

        figureOutPIDConstantsFor(backLeft);

        figureOutPIDConstantsFor(backRight);
    }

    private ProcessConsole swervePIDConsole;

    private void figureOutPIDConstantsFor(SwerveModule swerveModule) throws InterruptedException
    {
        taskPackage.add(swerveModule);
        taskPackage.run();

        Vector2D desiredRotation;

        swervePIDConsole = log.newProcessConsole(swerveModule.motorName + " PID");

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

        flow.msPause(3000);
    }
}
