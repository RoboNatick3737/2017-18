package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CompetitionOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.AutonomousBase;

@Autonomous(name="Red Bottom", group= Constants.FINAL_BOT_OPMODES)
public class RedBottom extends AutonomousBase
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
