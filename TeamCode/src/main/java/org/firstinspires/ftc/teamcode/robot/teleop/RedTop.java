package org.firstinspires.ftc.teamcode.robot.teleop;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompetitionOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.TeleopBase;

import hankextensions.input.HTButton;

@TeleOp(name="Red Top", group= Constants.FINAL_BOT_OPMODES)
public class RedTop extends TeleopBase
{
    @Override
    protected Alliance getAlliance()
    {
        return Alliance.RED;
    }

    @Override
    protected BalancePlateLocation getBalancePlateLocation()
    {
        return BalancePlateLocation.TOP;
    }
}
