package org.firstinspires.ftc.teamcode.robot.teleop;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.TeleopBase;

import hankextensions.input.HTButton;

@TeleOp(name="Final Teleop", group= Constants.FINAL_BOT_OPMODES)
public class FinalTeleop extends TeleopBase
{
    @Override
    public void onRunTeleop() throws InterruptedException
    {
        flipper.advanceStage(0);
        intake.stop();

        waitForStart();

        swerveDrive.setJoystickControlEnabled(true);
        swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        while (true)
        {
            // Update controllers
            C1.update();
            C2.update();

            // Update swerve drive
            swerveDrive.synchronousUpdate();

            // Control flipper
            if (C1.a.currentState == HTButton.ButtonState.JUST_TAPPED)
                flipper.advanceStage();

            // Use the ball knocker
            if (C1.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                ballKnocker.toggleKnocker();

            // Control intake
            if (C1.gamepad.left_bumper)
                intake.intake();
            else if (C1.gamepad.right_bumper)
                intake.expel();
            else
                intake.stop();

            // Toggle the harvester stage.
            if (C1.b.currentState == HTButton.ButtonState.JUST_TAPPED)
                intake.toggleHarvesterStage();

            // Control the lift.
            if (C2.gamepad.dpad_up)
                lift.up();
            else if (C2.gamepad.dpad_down)
                lift.down();
            else
                lift.stop();

            flow.yield();
        }
    }
}
