package org.firstinspires.ftc.teamcode.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.components.CryptoTracker;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.LinkedList;

import hankextensions.RobotCore;
import hankextensions.vision.opencv.OpenCVCam;
import visionanalysis.OpenCVJNIHooks;

@Autonomous(name="Optimized Cryptobox Vision", group= Constants.EXPERIMENTATION)
public class OptimizedCryptoboxVision extends RobotCore
{
    private OpenCVCam openCVCam;

    private ProcessConsole cameraProcessConsole;

    /**
     * This complex class takes care of
     */
    private CryptoTracker cryptoTracker;

    @Override
    protected void INITIALIZE() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        cryptoTracker = new CryptoTracker();
        openCVCam.start(cryptoTracker);

        cameraProcessConsole = log.newProcessConsole("Camera Process Console");
    }

    @Override
    protected void START() throws InterruptedException
    {
        while (true)
        {
            cameraProcessConsole.write(
                    "Forward offset: " + cryptoTracker.forwardOffset,
                    "Horizontal offset: " + cryptoTracker.horizontalOffset);
            flow.yield();
        }
    }
}
