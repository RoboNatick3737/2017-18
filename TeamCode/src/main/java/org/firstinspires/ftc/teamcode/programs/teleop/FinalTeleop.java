package org.firstinspires.ftc.teamcode.programs.teleop;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.HardwareBase;

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
        swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.ASYNCHRONOUS);

        while (true)
        {
            htGamepad1.update();
            htGamepad2.update();

            if (htGamepad1.a.currentState == HTButton.ButtonState.JUST_TAPPED)
                flipper.advanceStage();

            if (gamepad1.left_bumper)
                intake.intake();
            else if (gamepad1.right_bumper)
                intake.expel();
            else
                intake.stop();

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
