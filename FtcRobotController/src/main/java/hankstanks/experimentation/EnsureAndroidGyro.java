package hankstanks.experimentation;

import hankstanks.sdkextensions.Core;
import hankstanks.sdkextensions.hardware.AndroidGyro;
import hankstanks.sdkextensions.logging.ProcessConsole;
import hankstanks.sdkextensions.threading.Flow;

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
