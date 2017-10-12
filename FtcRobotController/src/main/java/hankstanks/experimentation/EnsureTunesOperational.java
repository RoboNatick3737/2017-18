package hankstanks.experimentation;

import hankstanks.sdkextensions.Core;
import hankstanks.sdkextensions.music.Tunes;
import hankstanks.sdkextensions.threading.Flow;

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
