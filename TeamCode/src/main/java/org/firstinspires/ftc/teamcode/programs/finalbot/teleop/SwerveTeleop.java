package org.firstinspires.ftc.teamcode.programs.finalbot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.finalbot.HardwareBase;
import org.firstinspires.ftc.teamcode.structs.Vector2D;

import hankextensions.logging.ProcessConsole;
import hankextensions.threading.Flow;

@TeleOp(name="Swerve Teleop", group= Constants.FINAL_BOT_OPMODES)
public class SwerveTeleop extends HardwareBase
{
    @Override
    protected void START() throws InterruptedException
    {
        swerveDrive.provideGamepad(gamepad1);

        while (true)
        {
            Flow.yield();
        }
    }
}
