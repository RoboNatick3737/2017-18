package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.TeleopBase;

@TeleOp(name="Blue Top", group= Constants.FINAL_BOT_OPMODES)
public class BlueTop extends TeleopBase
{
    @Override
    protected Alliance getAlliance()
    {
        return Alliance.BLUE;
    }

    @Override
    protected BalancePlateLocation getBalancePlateLocation()
    {
        return BalancePlateLocation.TOP;
    }
}
