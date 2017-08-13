package org.firstinspires.ftc.teamcode.sdkextensions.threading;

import org.firstinspires.ftc.teamcode.programs.Core;
import org.firstinspires.ftc.teamcode.sdkextensions.logging.Log;
import org.firstinspires.ftc.teamcode.sdkextensions.logging.ProcessConsole;

/**
 * NiFTSimpleTasks run in a single thread context, and are cycled through one by one in order to avoid counting toward the thread limit.
 */
public abstract class SimpleTask
{
    public final String taskName;
    protected ProcessConsole processConsole;
    public SimpleTask ()
    {
        this ("Unnamed Simple NiFTComplexTask");
    }
    public SimpleTask (String taskName)
    {
        this.taskName = taskName;
    }

    public void activate()
    {
        if (processConsole == null)
            processConsole = Core.log.newProcessConsole(taskName);
    }
    public void deactivate()
    {
        if (processConsole != null)
        {
            processConsole.destroy ();
            processConsole = null;
        }
    }

    //The long returned indicates the amount of time to wait before running the task again.
    public long nextRunTime = 0;
    protected abstract long onContinueTask () throws InterruptedException;
}