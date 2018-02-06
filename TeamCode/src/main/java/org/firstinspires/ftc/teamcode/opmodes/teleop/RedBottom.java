package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

@TeleOp(name="Red Bottom Teleop", group= OpModeDisplayGroups.FINAL_BOT_OPMODES)
public class RedBottom extends Teleop
{
    @Override
    public Alliance getAlliance()
    {
        return Alliance.RED;
    }

    @Override
    public BalancePlate getBalancePlate()
    {
        return BalancePlate.BOTTOM;
    }
}
