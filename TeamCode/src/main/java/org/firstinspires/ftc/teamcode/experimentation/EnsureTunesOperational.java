package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.EnhancedOpMode;
import hankextensions.music.Tunes;

@Autonomous(name="Pumping Beats", group="Experimentation")
public class EnsureTunesOperational extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        Tunes.play(Tunes.Option.USSR_Anthem);

        while (Tunes.playing())
            flow.yield();
    }
}
