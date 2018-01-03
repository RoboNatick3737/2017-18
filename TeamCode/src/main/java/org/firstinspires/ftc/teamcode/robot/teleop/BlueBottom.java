package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.TeleopBase;

@TeleOp(name="Blue Bottom", group= Constants.FINAL_BOT_OPMODES)
public class BlueBottom extends TeleopBase
{
    @Override
    protected Alliance getAlliance()
    {
        return Alliance.BLUE;
    }

    @Override
    protected BalancePlateLocation getBalancePlateLocation()
    {
        return BalancePlateLocation.BOTTOM;
    }
}
