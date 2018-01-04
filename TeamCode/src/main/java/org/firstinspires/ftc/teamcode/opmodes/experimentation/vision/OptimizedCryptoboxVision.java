package org.firstinspires.ftc.teamcode.opmodes.experimentation.vision;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import hankextensions.vision.opencv.analysis.CryptoboxTracker;

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
    private CryptoboxTracker cryptoboxTracker;

    @Override
    protected void onRun() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        cryptoboxTracker = new CryptoboxTracker();
        openCVCam.start(cryptoboxTracker);

        cameraProcessConsole = log.newProcessConsole("Camera Process Console");

        waitForStart();

        while (true)
        {
            cameraProcessConsole.write(
                    "Forward offset: " + cryptoboxTracker.forwardOffset,
                    "Horizontal offset: " + cryptoboxTracker.horizontalOffset);
            flow.yield();
        }
    }
}
