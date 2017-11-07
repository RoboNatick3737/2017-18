package hankextensions.threading;

import hankextensions.Core;

/**
 * This class is super important to running the program while making sure to check the user requested state of the program.  While running any wait method, the program will run essentially an idle() statement to check to see whether a stop was requested, and throw an InterruptedException in that event.
 *
 * Avoid sleep() statements since I don't think that they include stopRequested() checks.   NiFTFlow.msPause is better instead.
 */
public class Flow
{
    /**
     * This method "waits" for a given number of seconds by running yield() as long as necessary.
     *
     * @param ms the milliseconds that the program should wait for.
     * @throws InterruptedException the exception which indicates that the program needs to stop.
     */
    public static void msPause(long ms) throws InterruptedException
    {
        long startTime = System.currentTimeMillis ();
        while (System.currentTimeMillis () - startTime <= ms)
            yield();
    }

    /**
     * This method quickly checks to see whether the program needs to stop before allowing other threads to run.
     *
     * @throws InterruptedException
     */
    public static void yield() throws InterruptedException
    {
        if (Core.instance == null || Core.instance.isStopRequested())
            throw new InterruptedException("OpMode requested stop!");

        Thread.yield();
    }
}