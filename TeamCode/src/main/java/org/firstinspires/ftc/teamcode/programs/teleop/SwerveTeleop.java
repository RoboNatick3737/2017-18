package org.firstinspires.ftc.teamcode.programs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.HardwareBase;

@TeleOp(name="Swerve Teleop", group= Constants.FINAL_BOT_OPMODES)
public class SwerveTeleop extends HardwareBase
{
    @Override
    protected void START() throws InterruptedException
    {
        swerveDrive.provideGamepad(gamepad1);

        while (true)
        {
            flow.yield();
        }
    }
}
