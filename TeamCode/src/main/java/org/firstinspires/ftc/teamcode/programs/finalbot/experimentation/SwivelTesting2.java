package org.firstinspires.ftc.teamcode.programs.finalbot.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.finalbot.HardwareBase;
import org.firstinspires.ftc.teamcode.structs.Vector2D;

import hankextensions.threading.Flow;

@Autonomous(name="Swivel Testing 2", group=Constants.FINAL_BOT_EXPERIMENTATION)
public class SwivelTesting2 extends HardwareBase
{
    @Override
    protected void START() throws InterruptedException
    {
        while (true)
        {
            swerveDrive.alignWheelsTo(new Vector2D(1, 1).unit().orientToAngle(45));

            Flow.msPause(3000);
        }
    }
}
