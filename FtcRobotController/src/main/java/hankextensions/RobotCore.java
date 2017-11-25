package hankextensions;

import com.makiah.makiahsandroidlib.threading.Flow;
import com.makiah.makiahsandroidlib.threading.TaskParent;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import hankextensions.logging.TelemetryWrapper;
import hankextensions.phonesensors.AndroidGyro;
import hankextensions.vision.OpenCVCam;
import hankextensions.vision.VuforiaCam;
import hankextensions.music.Tunes;

/**
 * NiFTBase is the class from which all user OpModes should inherit.  With advanced error handling, it takes care of the scenarios in which the user requests an early stop, fails to take an error into account, etc.
 */
public abstract class RobotCore extends LinearOpMode implements TaskParent
{
    // Useful for other files which require custom initialization steps or components from this op mode which they cannot otherwise obtain.
    public static RobotCore instance;

    // Properties of the current RobotCore instance.
    public TelemetryWrapper log;
    public Flow flow;

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

            //REQUIRED in child classes.
            HARDWARE();

            //May be used in different programs.
            INITIALIZE();

            //Wait for the start button to be pressed.
            waitForStart ();

            // Since hitting stop makes waitForStart be bypassed.
            flow.yield();

            //This is where the child classes mainly differ in their instructions.
            START();
        }
        catch (InterruptedException e) {} //If this is caught, then the user requested program stop.
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
            // Stop playing tunes.
            Tunes.silence();

            // Disable both cameras (just in case)
            if (OpenCVCam.instance != null)
                OpenCVCam.instance.stop();
            if (VuforiaCam.instance != null)
                VuforiaCam.instance.stop();

            // Disable the Android gyro (in case the op mode didn't turn it off).
            if (AndroidGyro.instance != null)
                AndroidGyro.instance.quit();

            STOP();
        }
    }

    /**
     * Use by calling DcMotor dcMotor = NiFTInitializer.initialize(DcMotor.class, "name in config");
     */
    public <T extends HardwareDevice> T initHardwareDevice (Class<T> hardwareDevice, String name)
    {
        try
        {
            //Returns the last subclass (if this were a DcMotor it would pass back a Dc Motor.
            return hardwareDevice.cast (hardwareMap.get (name));
        }
        catch (Exception e) //There might be other exceptions that this throws, not entirely sure about which so I am general here.
        {
            throw new NullPointerException ("Couldn't find " + name + " in the configuration file!");
        }
    }

    /**
     * In this method, initialize all required hardware (motors, servos, etc.).
     */
    protected void HARDWARE() throws InterruptedException {};
    /**
     * In this method, do everything that needs to happen AFTER hardware initialization and during Init (like
     * gyro calibration).
     */
    protected void INITIALIZE() throws InterruptedException {}
    /**
     * Code everything that the robot needs to do upon play being tapped in here.
     */
    protected abstract void START() throws InterruptedException;
    /**
     * Any final actions that need to happen.
     */
    protected void STOP() {}
}