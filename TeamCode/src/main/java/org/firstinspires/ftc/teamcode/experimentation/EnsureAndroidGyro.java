package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.EnhancedOpMode;
import hankextensions.phonesensors.AndroidGyro;
import hankextensions.phonesensors.Gyro;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;

@Autonomous(name = "Test Android Gyro", group = "Experimentation")
public class EnsureAndroidGyro extends EnhancedOpMode
{
    private Gyro phoneGyro;

    @Override
    protected void onRun() throws InterruptedException {
        phoneGyro = new AndroidGyro();
        AndroidGyro.instance.start();

        waitForStart();

        ProcessConsole gyroConsole = log.newProcessConsole("Phone Gyro");

        while (true)
        {
            gyroConsole.write(
                    "X: " + phoneGyro.x(),
                    "Y: " + phoneGyro.y(),
                    "Z: " + phoneGyro.z()
            );

            flow.yield();
        }
    }
}
