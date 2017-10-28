package org.firstinspires.ftc.teamcode.programs.finalbot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.finalbot.HardwareBase;

@TeleOp(name="Swerve Teleop", group= Constants.FINAL_BOT_OPMODES)
public class SwerveTeleop extends HardwareBase
{
    @Override
    protected void INITIALIZE() throws InterruptedException
    {
        swerveDrive.startJoystickControl(gamepad1);
    }

    @Override
    protected void START() throws InterruptedException
    {
    }
}
