package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankstanks.hankextensions.Core;
import hankstanks.hankextensions.music.Tunes;
import hankstanks.hankextensions.threading.Flow;

@Autonomous(name="Pumping Beats", group="Experimentation")
public class EnsureTunesOperational extends Core
{
    protected void START() throws InterruptedException
    {
        Tunes.play(Tunes.Option.USSR_Anthem);

        while (Tunes.playing())
        {
            Flow.yield();
        }
    }
}
