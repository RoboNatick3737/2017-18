package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.GyroSensor;

import hankextensions.RobotCore;
import hankextensions.logging.TelemetryWrapper;
import hankextensions.structs.Vector2D;

import hankextensions.phonesensors.Gyro;

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
        RobotCore.instance.log.lines("Gyroscope calibrating...");

        //Pause to prevent odd errors in which it says it's configured but is actually LYING.
        RobotCore.instance.flow.msPause (1000);

        //Wait for gyro to finish calibrating.
        while (sensor.isCalibrating())
            RobotCore.instance.flow.msPause (50);

        //Zero gyro heading.
        if (zeroHeading)
            zero();

        RobotCore.instance.log.lines("Gyroscope calibration complete!");
    }

    /**
     * calibrate() overload to satisfy Gyro interface.
     */
    public void calibrate() throws InterruptedException
    {
        calibrate(true);
    }

    //Just resets the gyro.
    public void zero() throws InterruptedException
    {
        RobotCore.instance.flow.msPause (400);
        sensor.resetZAxisIntegrator();
        RobotCore.instance.flow.msPause (400);
    }

    /**
     * Returns a gyro value between 0 to 360.
     */
    public double x()
    {
        return 0;
    }

    public double y()
    {
        return 0;
    }

    public double z()
    {
        return Vector2D.clampAngle(sensor.getHeading());
    }
}
