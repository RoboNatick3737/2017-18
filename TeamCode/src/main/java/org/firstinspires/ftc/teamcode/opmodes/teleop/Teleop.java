package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveDrive;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveModule;

import hankextensions.EnhancedOpMode;
import hankextensions.input.HTButton;

public abstract class Teleop extends EnhancedOpMode implements CompetitionProgram
{
    /**
     * The teleop controls for our swerve drive and such.
     */
    @Override
    protected final void onRun() throws InterruptedException
    {
        // Check the battery.
        double batteryCoefficient = getBatteryCoefficient();

        Robot robot = new Robot(hardware, Robot.ControlMode.TELEOP);

        // Enable logging TODO remove
        for (SwerveModule module : robot.swerveDrive.swerveModules)
            module.setEnableLogging(true);

        // Synchronous teleop
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);
        robot.swerveDrive.setControlMethod(SwerveDrive.ControlMethod.TANK_DRIVE);

        // Init robot hardware.
        robot.flipper.advanceStage(0);
        robot.intake.stop();

        waitForStart();

        robot.gyro.startAntiDrift();

        while (true)
        {
            // Update controllers
            C1.update();
            C2.update();

            // Update swerve drive
            if (C1.b.currentState == HTButton.ButtonState.JUST_TAPPED)
            {
                if (robot.swerveDrive.getSwerveSpeedMode() == SwerveDrive.SwerveSpeedMode.FAST)
                    robot.swerveDrive.setSwerveSpeedMode(SwerveDrive.SwerveSpeedMode.SLOW);
                else
                    robot.swerveDrive.setSwerveSpeedMode(SwerveDrive.SwerveSpeedMode.FAST);
            }
            robot.swerveDrive.synchronousUpdate();

            // Control flipper
            if (C1.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.flipper.advanceStage();
            robot.flipper.update(); // in case we're doing the timed lift thing.

            if (C1.y.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.lights.toggleLights();

            // Control intake
            if (C1.gamepad.left_bumper)
                robot.intake.expel();
            else if (C1.gamepad.right_bumper)
                robot.intake.intake();
            else if (C1.gamepad.left_trigger > .03 || C1.gamepad.right_trigger > .03)
            {
                robot.intake.leftIntake(C1.gamepad.left_trigger);
                robot.intake.rightIntake(C1.gamepad.right_trigger);
                robot.intake.secondaryIntake(1);
            }
            else
                robot.intake.stop();

            // Control the lift.
            if (C2.gamepad.dpad_up)
                robot.lift.up();
            else if (C2.gamepad.dpad_down)
                robot.lift.down();
            else
                robot.lift.stop();

            // Control ball knocker (debugging)
            if (C2.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.ballKnocker.toggleDescender();

            if (C2.y.currentState == HTButton.ButtonState.JUST_TAPPED)
                robot.ballKnocker.setKnockerTo(Math.random() > 0.5 ? BallKnocker.KnockerPosition.LEFT : BallKnocker.KnockerPosition.RIGHT);

            // Controls the relic arm
//            if (C2.y.currentState == HTButton.ButtonState.JUST_TAPPED)
//                robot.relicSystem.toggleGrabber();

//            robot.relicSystem.variableExtension(C2.gamepad.right_trigger, C2.gamepad.left_trigger);

//            if (C2.gamepad.dpad_left)
//                robot.relicSystem.rotate(false);
//            else if (C2.gamepad.dpad_right)
//                robot.relicSystem.rotate(true);
//            else
//                robot.relicSystem.stopRotator();

            flow.yield();
        }
    }
}
