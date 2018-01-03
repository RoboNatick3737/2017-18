package org.firstinspires.ftc.teamcode.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.components.CryptoTracker;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Optimized Cryptobox Vision", group= Constants.EXPERIMENTATION)
public class OptimizedCryptoboxVision extends EnhancedOpMode
{
    private OpenCVCam openCVCam;

    private ProcessConsole cameraProcessConsole;

    /**
     * This complex class takes care of
     */
    private CryptoTracker cryptoTracker;

    @Override
    protected void onRun() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        cryptoTracker = new CryptoTracker();
        openCVCam.start(cryptoTracker);

        cameraProcessConsole = log.newProcessConsole("Camera Process Console");

        waitForStart();

        while (true)
        {
            cameraProcessConsole.write(
                    "Forward offset: " + cryptoTracker.forwardOffset,
                    "Horizontal offset: " + cryptoTracker.horizontalOffset);
            flow.yield();
        }
    }
}
