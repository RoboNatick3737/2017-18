package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;

@Autonomous(name="Blue Top Auto", group= Constants.FINAL_BOT_OPMODES)
public class BlueTop extends AutonomousBase
{
    @Override
    public Alliance getAlliance()
    {
        return Alliance.BLUE;
    }

    @Override
    public BalancePlate getBalancePlate()
    {
        return BalancePlate.TOP;
    }
}
