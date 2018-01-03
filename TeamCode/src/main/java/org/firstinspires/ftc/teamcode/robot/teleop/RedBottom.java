package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.TeleopBase;

@TeleOp(name="Red Bottom", group= Constants.FINAL_BOT_OPMODES)
public class RedBottom extends TeleopBase
{
    @Override
    protected Alliance getAlliance()
    {
        return Alliance.RED;
    }

    @Override
    protected BalancePlateLocation getBalancePlateLocation()
    {
        return BalancePlateLocation.BOTTOM;
    }
}
