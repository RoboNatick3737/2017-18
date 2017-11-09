package hankextensions.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

import hankextensions.Core;
import hankextensions.threading.ComplexTask;
import hankextensions.threading.Flow;

/**
 * The Advanced Console is an easy way to visualize a large number of tasks in parallel without having to rely
 * on superhuman vision.  It also supports sequential logging in the same window.
 *
 * This console uses a task to update its content so that it isn't jumpy when displayed on the driver station.
 *
 * Local so that resetting is not an issue.
 */

public class Log
{
    public static Log instance;

    /*-- USE TO OUTPUT DATA IN A SLIGHTLY BETTER WAY THAT LINEAR OP MODES PROVIDE --*/
    private ArrayList<String> sequentialConsoleData; //Lines being added and removed.
    private ArrayList<ProcessConsole> privateProcessConsoles;
    private Telemetry mainTelemetry;

    private static final int MAX_SEQUENTIAL_LINES = 13;

    /**
     * Resets the entire console with empty content.
     */
    public Log() throws InterruptedException
    {
        instance = this;

        mainTelemetry = Core.instance.telemetry;

        //Initialize required components.
        sequentialConsoleData = new ArrayList<>();
        privateProcessConsoles = new ArrayList<>();
    }

    public void close()
    {
        instance = null;
    }

    public void lines(String... newLines)
    {
        //Add new line at beginning of the lines.
        for (String line: newLines)
            sequentialConsoleData.add (0, line);
        //If there is more than 5 lines there, remove one.
        while (sequentialConsoleData.size () > MAX_SEQUENTIAL_LINES)
            sequentialConsoleData.remove (MAX_SEQUENTIAL_LINES);
    }

    public void appendToLastSequentialLine (String toAppend)
    {
        String result = sequentialConsoleData.get (0) + toAppend;
        sequentialConsoleData.remove (0);
        sequentialConsoleData.add (0, result);
    }

    /**
     * To get a private process console, create a new Log.ProcessConsole(<name here>) and then run write() to provide new content.
     */

    public ProcessConsole newProcessConsole(String name)
    {
        return new ProcessConsole(name, privateProcessConsoles);
    }

    /**
     * The task which updates the console at a fairly slow rate but your eye can't tell the difference.
     */
    private class ConsoleUpdater extends ComplexTask
    {
        public ConsoleUpdater() {
            super("Console Updater");
        }

        @Override
        protected void onDoTask () throws InterruptedException
        {
            while (true)
            {
                rebuildConsole ();
                Flow.msPause(300);
            }
        }
    }

    private ConsoleUpdater consoleUpdaterInstance = null;

    /**
     * Creates a new console updater instance and runs it.
     */
    public void startConsoleUpdater ()
    {
        if (consoleUpdaterInstance == null)
        {
            consoleUpdaterInstance = new ConsoleUpdater ();
            consoleUpdaterInstance.run();
        }
    }

    /**
     * Nullifies a new console instance and stops it.
     */
    public void stopConsoleUpdater ()
    {
        if (consoleUpdaterInstance != null)
        {
            consoleUpdaterInstance.stop ();
            consoleUpdaterInstance = null;
        }
    }

    /**
     * Rebuilds the whole console (call minimally, allow the task to take care of it.)
     */
    public void rebuildConsole ()
    {
        if (mainTelemetry != null)
        {
            //Clear all lines.
            mainTelemetry.update ();

            //Add all private console data.
            for (ProcessConsole pConsole : privateProcessConsoles)
            {
                mainTelemetry.addLine ("----- " + pConsole.processName + " -----");

                for (String line : pConsole.processData)
                    mainTelemetry.addLine (line);

                mainTelemetry.addLine ("");
            }

            mainTelemetry.addLine ("----- Sequential Data -----");
            for (String line : sequentialConsoleData)
            {
                mainTelemetry.addLine (line);
            }

            //Refresh the console with this new data.
            mainTelemetry.update ();
        }
        //Otherwise it just gets queued in the ArrayList.
    }
}