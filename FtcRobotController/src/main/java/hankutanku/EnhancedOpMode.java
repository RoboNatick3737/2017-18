package hankutanku;

import dude.makiah.androidlib.threading.Flow;
import dude.makiah.androidlib.threading.TaskParent;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

import hankutanku.hardware.HardwareInitializer;
import hankutanku.input.HTGamepad;
import hankutanku.logging.TelemetryWrapper;
import hankutanku.phonesensors.AndroidGyro;
import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.vuforia.VuforiaCam;
import hankutanku.music.Tunes;

/**
 * EnhancedOpMode is the class from which all user OpModes should inherit.  With advanced error handling, it takes care of the scenarios in which the user requests an early stop, fails to take an error into account, etc.
 */
public abstract class EnhancedOpMode extends LinearOpMode implements TaskParent
{
    // Useful for other files which require custom initialization steps or components from this op mode which they cannot otherwise obtain.
    public static EnhancedOpMode instance;

    // Auto or teleop.
    public enum AutoOrTeleop { AUTONOMOUS, TELEOP }

    /**
     * Returns the current battery voltage as a coefficient between 0 and 1, defined by a minimum
     * and maximum voltage.
     * @param minVolts  The minimum voltage (if below this, it'll signal the driver).
     * @param maxVolts  The maximum voltage (if above this, it'll just ignore it).
     */
    private void updateBatteryCoefficient(double minVolts, double maxVolts)
    {
        // Slightly changes OpMode progression.
        String voltageCheck = FtcEventLoopHandler.latestBatterySend;
        double batteryCoefficient = 0.5; // between 1 (14.1V) and 0 (12.2V).
        if (!voltageCheck.equals("")) // something weird happened?
        {
            double batteryVoltageCheck = 0;
            try
            {
                batteryVoltageCheck = Double.parseDouble(voltageCheck);
            }
            catch (Exception e)
            {
                return;
            }

            if (batteryVoltageCheck < minVolts)
                AppUtil.getInstance().showToast(UILocation.BOTH, "Change the battery >:(");

            batteryCoefficient = (batteryVoltageCheck - minVolts) / (maxVolts - minVolts);
            batteryCoefficient = Range.clip(batteryCoefficient, 0, 1);

            if (instance != null)
                instance.log.lines("Battery coefficient is " + batteryCoefficient);
        }

        this.batteryCoefficient = batteryCoefficient;
    }
    protected double batteryCoefficient = 0;

    // Properties of the current RobotCore instance.
    public TelemetryWrapper log;
    public Flow flow;

    // Two pointers to the gamepads for the purpose of concision.
    protected HTGamepad C1, C2;

    protected HardwareInitializer hardware;

    /**
     * As a TaskParent, RobotCore must implement this method.
     */
    public boolean isTaskActive()
    {
        return !isStopRequested();
    }

    /**
     * runOpMode() is the method called by LinearOpMode to start the program, but is really low-level.  What this method does is split the sequence into a set of steps which every autonomous program should include, while also observing errors and either stopping the code or outputting them based on their severity.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode () throws InterruptedException
    {
        try
        {
            // Singleton robot.
            instance = this;

            //Classes such as NiFTMusic require this so that they can get the context they require.
            flow = new Flow(this);
            log = new TelemetryWrapper(telemetry);

            // Update the battery coefficient.
            updateBatteryCoefficient(12.4, 14.1);

            // Init the gamepads
            C1 = new HTGamepad(gamepad1, HTGamepad.ControllerID.CONTROLLER_1);
            C2 = new HTGamepad(gamepad2, HTGamepad.ControllerID.CONTROLLER_2);

            // Hardware initializer
            hardware = new HardwareInitializer(hardwareMap);

            // Run the OpMode.
            onRun();
        }
        catch (InterruptedException e) {} //If this is caught, then the user requested program stop.
        catch (IllegalStateException e)
        {
            log.lines("Weird exception happened");
        }
        catch (Exception e) //If this is caught, it wasn't an InterruptedException and wasn't requested, so the user is notified.
        {
            log.lines("UH OH!  An error was just thrown!");
            log.lines(e.getMessage ());
            log.lines("Will end upon tapping stop...");

            //Wait until stop is requested.
            try
            {
                while (true)
                    flow.yield ();
            }
            catch (InterruptedException e2) {} //The user has read the message and stops the program.
        }
        finally //Occurs after all possible endings.
        {
            // Disable both cameras (just in case)
            if (OpenCVCam.instance != null)
                OpenCVCam.instance.stop();
            if (VuforiaCam.instance != null)
                VuforiaCam.instance.stop(flow);

            // Disable the Android gyro (in case the op mode didn't turn it off).
            if (AndroidGyro.instance != null)
                AndroidGyro.instance.quit();

            // Stop playing tunes.
            Tunes.silence();

            onStop();
        }
    }

    /**
     * Code everything that the robot needs to do upon play being tapped in here.
     */
    protected abstract void onRun() throws InterruptedException;
    /**
     * Any final actions that need to happen.
     */
    protected void onStop() {}
}