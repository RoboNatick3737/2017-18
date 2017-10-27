package org.firstinspires.ftc.teamcode.programs.prelimbot.programs.autonomous;

import org.firstinspires.ftc.teamcode.programs.prelimbot.HardwareBase;
import hankextensions.music.Tunes;

public abstract class BaseAuto extends HardwareBase
{
    private void driveSomewhere()
    {
    }

    private void hailTheSoviets()
    {
        Tunes.play(Tunes.Option.USSR_Anthem);
    }

    private void saySomething()
    {
        log.lines("I'm giving up on you");
    }
}
