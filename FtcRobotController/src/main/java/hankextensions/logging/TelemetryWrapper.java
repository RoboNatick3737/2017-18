package hankextensions.logging;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import hankextensions.RobotCore;

/**
 * Uses my lib to visualize task states on the driver station.
 */
public class TelemetryWrapper extends LoggingBase
{
    private final Telemetry mainTelemetry;

    /**
     * Resets the entire console with empty content.
     */
    public TelemetryWrapper(Telemetry mainTelemetry) throws InterruptedException
    {
        super(RobotCore.instance);

        // Set instance and static components.
        this.mainTelemetry = mainTelemetry;

        // Start the updater ASAP.
        this.run();
    }

    /**
     * Rebuilds the whole telemetry console (call minimally, allow the task to take care of it.)
     */
    @Override
    protected void refreshOnScreenConsole()
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