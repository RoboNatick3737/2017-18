package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.GyroSensor;

import hankextensions.phonesensors.Gyro;
import hankextensions.threading.Flow;

/**
 * Encapsulates the gyro sensor in an easy to access set of methods.
 */
public class MRGyro implements Gyro
{
    public final GyroSensor sensor;

    public MRGyro(GyroSensor sensor) throws InterruptedException
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

    /**
     * calibrate() overload to satisfy Gyro interface.
     */
    public void calibrate() throws InterruptedException
    {
        calibrate(false);
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
    public double x()
    {
        //Get the heading.
        return sensor.getHeading ();
    }

    public double y()
    {
        return 0;
    }

    public double z()
    {
        return 0;
    }
}
