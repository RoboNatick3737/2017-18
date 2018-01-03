package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.AutonomousBase;

@Autonomous(name="Red Top", group= Constants.FINAL_BOT_OPMODES)
public class RedTop extends AutonomousBase
{
    @Override
    protected void onRunAutonomous() throws InterruptedException
    {
    }

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected BalancePlateLocation getBalancePlateLocation() {
        return BalancePlateLocation.TOP;
    }
}
