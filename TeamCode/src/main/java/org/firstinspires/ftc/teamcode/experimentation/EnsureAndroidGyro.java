package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankstanks.hankextensions.Core;
import hankstanks.hankextensions.hardware.AndroidGyro;
import hankstanks.hankextensions.logging.ProcessConsole;
import hankstanks.hankextensions.threading.Flow;

@Autonomous(name = "Test Android Gyro", group = "Experimentation")
public class EnsureAndroidGyro extends Core
{
    AndroidGyro phoneGyro;

    @Override
    protected void INITIALIZE() throws InterruptedException {
        phoneGyro = new AndroidGyro(hardwareMap.appContext);
    }

    protected void START() throws InterruptedException
    {
        ProcessConsole gyroConsole = log.newProcessConsole("Phone Gyro");

        while (true)
        {
            gyroConsole.write(
                    "X: " + phoneGyro.getX(),
                    "Y: " + phoneGyro.getY(),
                    "Z: " + phoneGyro.getZ()
            );

            Flow.yield();
        }
    }
}
