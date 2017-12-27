package org.firstinspires.ftc.teamcode.robot.teleop;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

import hankextensions.input.HTButton;

@TeleOp(name="Final Teleop", group= Constants.FINAL_BOT_OPMODES)
public class FinalTeleop extends TeleopBase
{
    @Override
    public void INITIALIZE() throws InterruptedException
    {
        flipper.advanceStage(0);
        intake.stop();
    }

    @Override
    protected void START() throws InterruptedException
    {
        swerveDrive.provideGamepad(gamepad1);
        swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        while (true)
        {
            htGamepad1.update();
            htGamepad2.update();

            swerveDrive.synchronousUpdate();

            if (htGamepad1.a.currentState == HTButton.ButtonState.JUST_TAPPED)
                flipper.advanceStage();

            if (gamepad1.left_bumper)
                intake.intake();
            else if (gamepad1.right_bumper)
                intake.expel();
            else
                intake.stop();

            if (htGamepad1.b.currentState == HTButton.ButtonState.JUST_TAPPED)
                intake.advanceHarvesterStage();

            if (gamepad2.dpad_up)
                lift.up();
            else if (gamepad2.dpad_down)
                lift.down();
            else
                lift.stop();

            flow.yield();
        }
    }
}
