package org.firstinspires.ftc.teamcode.robot.hardware;

import dude.makiah.androidlib.logging.LoggingBase;
import com.qualcomm.robotcore.hardware.GyroSensor;

import hankutanku.EnhancedOpMode;
import hankutanku.math.Vector2D;

import hankutanku.phonesensors.Gyro;

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
        LoggingBase.instance.lines("Gyroscope calibrating...");

        //Pause to prevent odd errors in which it says it's configured but is actually LYING.
        EnhancedOpMode.instance.flow.msPause (1000);

        //Wait for gyro to finish calibrating.
        while (sensor.isCalibrating())
            EnhancedOpMode.instance.flow.msPause (50);

        //Zero gyro heading.
        if (zeroHeading)
            zero();

        LoggingBase.instance.lines("Gyroscope calibration complete!");
    }

    @Override
    public void initAntiDrift() throws InterruptedException
    {
        calibrate(true);
    }

    @Override
    public void startAntiDrift() throws InterruptedException {}

    @Override
    public void applyOffset(double offset) {

    }

    //Just resets the gyro.
    public void zero() throws InterruptedException
    {
        EnhancedOpMode.instance.flow.msPause (400);
        sensor.resetZAxisIntegrator();
        EnhancedOpMode.instance.flow.msPause (400);
    }

    @Override
    public double getHeading()
    {
        return Vector2D.clampAngle(sensor.getHeading());
    }
}
