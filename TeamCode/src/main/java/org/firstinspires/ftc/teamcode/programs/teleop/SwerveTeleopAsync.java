package org.firstinspires.ftc.teamcode.programs.teleop;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.HardwareBase;
import org.firstinspires.ftc.teamcode.programs.hardware.SwerveDrive;

@TeleOp(name="Swerve Teleop — Async", group= Constants.FINAL_BOT_OPMODES)
public class SwerveTeleopAsync extends HardwareBase
{
    @Override
    protected void START() throws InterruptedException
    {
        swerveDrive.provideGamepad(gamepad1);
        swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.ASYNCHRONOUS);

        while (true)
            flow.yield();
    }
}
