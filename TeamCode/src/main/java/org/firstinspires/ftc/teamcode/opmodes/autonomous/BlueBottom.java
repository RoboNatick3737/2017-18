package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;

@Autonomous(name="Blue Bottom Auto", group= Constants.FINAL_BOT_OPMODES)
public class BlueBottom extends AutonomousBase
{
    @Override
    public Alliance getAlliance() {
        return null;
    }

    @Override
    public BalancePlate getBalancePlate() {
        return null;
    }
}
