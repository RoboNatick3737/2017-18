package org.firstinspires.ftc.teamcode.programs.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.programs.Core;
import org.firstinspires.ftc.teamcode.sdkextensions.music.Tunes;
import org.firstinspires.ftc.teamcode.sdkextensions.threading.Flow;

@Autonomous(name="Ensure Operational Tunes", group="Experimentation")
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
