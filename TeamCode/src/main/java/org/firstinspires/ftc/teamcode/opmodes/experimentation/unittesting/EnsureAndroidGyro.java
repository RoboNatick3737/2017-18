package org.firstinspires.ftc.teamcode.opmodes.experimentation.unittesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankutanku.EnhancedOpMode;
import hankutanku.phonesensors.AndroidGyro;

import dude.makiah.androidlib.logging.ProcessConsole;

@Autonomous(name = "Test Android Gyro", group = "Experimentation")
public class EnsureAndroidGyro extends EnhancedOpMode
{
    private AndroidGyro phoneGyro;

    @Override
    protected void onRun() throws InterruptedException
    {
        phoneGyro = new AndroidGyro();
        AndroidGyro.instance.start();
        phoneGyro.initAntiDrift();

        waitForStart();

        phoneGyro.startAntiDrift();

        ProcessConsole gyroConsole = log.newProcessConsole("Phone Gyro");

        while (true)
        {
            gyroConsole.write(
                    "X: " + phoneGyro.x,
                    "Y: " + phoneGyro.y,
                    "Z: " + phoneGyro.z,
                    "Heading: " + phoneGyro.getHeading()
            );

            flow.yield();
        }
    }
}
