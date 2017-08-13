package org.firstinspires.ftc.teamcode.sdkextensions.threading;

import org.firstinspires.ftc.teamcode.sdkextensions.logging.Log;
import org.firstinspires.ftc.teamcode.sdkextensions.logging.ProcessConsole;

import java.util.ArrayList;

/**
 * Used to divvy up SimpleTasks into certain groups.
 */
public class SimpleTaskPackage
{
    /**
     * Where they would generally go.
     */
    public static SimpleTaskPackage generalTaskPool;

    /**
     * Might as well include a name for the group of tasks!
     */
    public final String groupName;
    private ProcessConsole processConsole;
    public SimpleTaskPackage (String groupName)
    {
        this(groupName, null);
    }
    public SimpleTaskPackage(String groupName, SimpleTask... tasks)
    {
        this.groupName = groupName;

        //Populate task list.
        for (SimpleTask task : tasks)
            add(task);
    }

    /**
     * Place tasks in here, which will be run by the complex task class nested in this class.
     */
    private ArrayList<SimpleTask> taskList = new ArrayList<> ();
    public void add(SimpleTask simpleTask)
    {
        taskList.add (simpleTask);
        simpleTask.activate ();
    }
    public void remove(SimpleTask simpleTask)
    {
        taskList.remove (simpleTask);
        simpleTask.deactivate ();
    }

    /**
     * The SimpleTaskUpdater is a ComplexTask which takes up one AsyncTask spot but runs more than one task.
     */
    private class SimpleTaskUpdater extends ComplexTask
    {
        public SimpleTaskUpdater(String taskName)
        {
            super(taskName);
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
                    if (task.nextRunTime < System.currentTimeMillis ())
                        task.nextRunTime = task.onContinueTask () + System.currentTimeMillis ();
                }

                //Exit program if stop requested, otherwise yield to other threads.
                Flow.yield ();
            }
        }
    }
    private SimpleTaskUpdater taskUpdaterInstance = null;
    public void startTaskUpdater()
    {
        if (taskUpdaterInstance == null)
        {
            taskUpdaterInstance = new SimpleTaskUpdater (groupName + " Task");
            taskUpdaterInstance.run ();
        }
    }
    public void stopTaskUpdater()
    {
        if (taskUpdaterInstance != null)
        {
            taskUpdaterInstance.stop ();
            taskUpdaterInstance = null;
        }
    }
}