package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.Core;
import hankextensions.phonesensors.AndroidGyro;
import hankextensions.phonesensors.Gyro;

import hankextensions.logging.ProcessConsole;
import hankextensions.threading.Flow;

@Autonomous(name = "Test Android Gyro", group = "Experimentation")
public class EnsureAndroidGyro extends Core
{
    Gyro phoneGyro;

    @Override
    protected void INITIALIZE() throws InterruptedException {
        phoneGyro = new AndroidGyro();
        AndroidGyro.instance.start();
    }

    protected void START() throws InterruptedException
    {
        ProcessConsole gyroConsole = log.newProcessConsole("Phone Gyro");

        while (true)
        {
            gyroConsole.write(
                    "X: " + phoneGyro.x(),
                    "Y: " + phoneGyro.y(),
                    "Z: " + phoneGyro.z()
            );

            Flow.yield();
        }
    }
}
