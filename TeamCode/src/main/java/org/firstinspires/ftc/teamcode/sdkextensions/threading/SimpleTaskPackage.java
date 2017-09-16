/**
 * Used to divvy up SimpleTasks into certain groups, depending on their function.
 */

package org.firstinspires.ftc.teamcode.sdkextensions.threading;

import org.firstinspires.ftc.teamcode.programs.Core;
import org.firstinspires.ftc.teamcode.sdkextensions.logging.ProcessConsole;

import java.util.ArrayList;

public class SimpleTaskPackage
{
    public final String groupName;
    private ProcessConsole processConsole;
    public SimpleTaskPackage (String groupName)
    {
        this(groupName, null);
    }
    public SimpleTaskPackage(String groupName, SimpleTask... tasks)
    {
        processConsole = Core.log.newProcessConsole(groupName);

        this.groupName = groupName;

        //Populate task list.
        for (SimpleTask task : tasks)
        {
            add(task);
            task.containingPackage = this;
        }
    }

    /**
     * Place tasks in here, which will be run by the complex task class nested in this class.
     */
    private ArrayList<SimpleTask> taskList = new ArrayList<> ();
    public void add(SimpleTask simpleTask)
    {
        taskList.add (simpleTask);
    }
    public void remove(SimpleTask simpleTask)
    {
        taskList.remove (simpleTask);
    }

    /**
     * The TaskPackageRunner is a ComplexTask which just loops through the list of simple tasks that it needs to run and runs each
     * depending on the pause they ask for.
     */
    private class TaskPackageRunner extends ComplexTask
    {
        public TaskPackageRunner()
        {
            super(groupName + " Task Package");
        }

        @Override
        protected void onDoTask () throws InterruptedException
        {
            while (true)
            {
                //Cycle through whole list of tasks and run all that can be run.
                for (int i = 0; i < taskList.size (); i++)
                {
                    //Run if possible.
                    SimpleTask task = taskList.get(i);
                    if (task.isRunning() && task.nextRunTime < System.currentTimeMillis ())
                        task.nextRunTime = task.onContinueTask () + System.currentTimeMillis ();
                }

                //Exit program if stop requested, otherwise yield to other threads.
                Flow.yield ();
            }
        }
    }
    private TaskPackageRunner taskPackageRunner = null;
    public void start()
    {
        if (taskPackageRunner == null)
        {
            taskPackageRunner = new TaskPackageRunner();
            taskPackageRunner.run ();
        }
    }
    public void pause()
    {
        if (taskPackageRunner != null)
        {
            taskPackageRunner.stop ();
            taskPackageRunner = null;
        }
    }
}