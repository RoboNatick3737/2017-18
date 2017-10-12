package org.firstinspires.ftc.teamcode.experimentation;

import hankstanks.hankextensions.Core;
import hankstanks.hankextensions.music.Tunes;
import hankstanks.hankextensions.threading.Flow;

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
