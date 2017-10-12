package hankstanks.hankextensions.hardware;

import com.qualcomm.robotcore.hardware.GyroSensor;

import hankstanks.hankextensions.threading.Flow;

/**
 * Encapsulates the gyro sensor in an easy to access set of methods.
 */
public class Gyro
{
    public final GyroSensor sensor;

    public Gyro (GyroSensor sensor) throws InterruptedException
    {
        this.sensor = sensor;

        calibrate (true);
    }

    /**
     * Calibrates the gyro and optionally zeroes the heading.
     *
     * @param zeroHeading if true, it zeroes the heading as well.
     * @throws InterruptedException
     */
    public void calibrate(boolean zeroHeading) throws InterruptedException
    {
        //Pause to prevent odd errors in which it says it's configured but is actually LYING.
        Flow.msPause (1000);

        //Wait for gyro to finish calibrating.
        while (sensor.isCalibrating())
            Flow.msPause (50);

        //Zero gyro heading.
        if (zeroHeading)
            zero();
    }

    //Just resets the gyro.
    public void zero() throws InterruptedException
    {
        Flow.msPause (400);
        sensor.resetZAxisIntegrator();
        Flow.msPause (400);
    }

    /**
     * Returns a gyro value between 0 to 360.
     */
    public int heading()
    {
        //Get the heading.
        return sensor.getHeading ();
    }
}
