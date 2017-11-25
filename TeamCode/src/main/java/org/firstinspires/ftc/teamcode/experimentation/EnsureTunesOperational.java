package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.RobotCore;
import hankextensions.music.Tunes;

@Autonomous(name="Pumping Beats", group="Experimentation")
public class EnsureTunesOperational extends RobotCore
{
    protected void START() throws InterruptedException
    {
        Tunes.play(Tunes.Option.USSR_Anthem);

        while (Tunes.playing())
            flow.yield();
    }
}
