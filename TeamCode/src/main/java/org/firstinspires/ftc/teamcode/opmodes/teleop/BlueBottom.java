package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.AutonomousBase;

@Autonomous(name="Blue Bottom Teleop", group= Constants.FINAL_BOT_OPMODES)
public class BlueBottom extends TeleopBase
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
