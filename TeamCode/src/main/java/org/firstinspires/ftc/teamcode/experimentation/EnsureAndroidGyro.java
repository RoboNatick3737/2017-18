package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.programs.Core;
import org.firstinspires.ftc.teamcode.sdkextensions.hardware.AndroidGyro;
import org.firstinspires.ftc.teamcode.sdkextensions.logging.ProcessConsole;
import org.firstinspires.ftc.teamcode.sdkextensions.music.Tunes;
import org.firstinspires.ftc.teamcode.sdkextensions.threading.Flow;

@Autonomous(name="Ensure Android Gyro", group="Experimentation")
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
        }
    }
}
