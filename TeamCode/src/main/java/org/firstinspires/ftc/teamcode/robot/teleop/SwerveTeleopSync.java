package org.firstinspires.ftc.teamcode.robot.teleop;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.HardwareBase;

@TeleOp(name="Swerve Teleop — Sync", group= Constants.FINAL_BOT_OPMODES)
public class SwerveTeleopSync extends HardwareBase
{
    @Override
    protected void START() throws InterruptedException
    {
        swerveDrive.provideGamepad(gamepad1);
        swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        while (true)
        {
            swerveDrive.synchronousUpdate();
            flow.yield();
        }
    }
}
