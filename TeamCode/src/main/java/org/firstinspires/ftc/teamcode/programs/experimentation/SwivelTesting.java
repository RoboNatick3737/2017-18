package org.firstinspires.ftc.teamcode.programs.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.HardwareBase;

@Autonomous(name="Swivel Testing", group=Constants.FINAL_BOT_EXPERIMENTATION)
public class SwivelTesting extends HardwareBase
{
    @Override
    protected void START() throws InterruptedException
    {
        while (true) {
            swerveDrive.setDesiredHeading(Math.random() * 359);

            flow.msPause(3000);
        }
    }
}
