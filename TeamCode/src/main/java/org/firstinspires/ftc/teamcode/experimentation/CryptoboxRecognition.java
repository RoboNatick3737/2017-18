package org.firstinspires.ftc.teamcode.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

import hankextensions.RobotCore;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Cryptobox Recognition", group= Constants.EXPERIMENTATION)
public class CryptoboxRecognition extends RobotCore implements CameraBridgeViewBase.CvCameraViewListener
{
    private OpenCVCam openCVCam;

    private LinkedList<Mat> channels;
    private Mat blueMask, whiteMask, finalMask;

    private ProcessConsole cameraProcessConsole;

    private Size cameraFrameSize;

    @Override
    protected void INITIALIZE() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        openCVCam.start(this);
    }

    @Override
    protected void START() throws InterruptedException
    {
        while (true)
            flow.yield();
    }

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        cameraFrameSize = new Size(width, height);

        channels = new LinkedList<>();
        blueMask = new Mat();
        whiteMask = new Mat();
        finalMask = new Mat();
    }

    @Override
    public void onCameraViewStopped()
    {
        blueMask.release();
        whiteMask.release();
        finalMask.release();
    }

    @Override
    public Mat onCameraFrame(Mat rgb)
    {
        // Remove the alpha channel
        Imgproc.cvtColor(rgb, rgb, Imgproc.COLOR_RGBA2RGB);
//
//        // Get blue mask.
//        Core.inRange(rgb, new Scalar(10, 20, 60), new Scalar(120, 160, 255), blueMask);

        // Get white mask (the white stripes).
        Core.inRange(rgb, new Scalar(125, 125, 125), new Scalar(255, 255, 255), whiteMask);

        // Get the final mask (the combination of the two
//        Core.bitwise_or(blueMask, whiteMask, finalMask);
//
//        // Invert the final mask.
//        Core.bitwise_not(finalMask, finalMask);
//
//        // Now convert the image to grayscale and generate the Gaussian adaptive threshold over the mask.
//        Imgproc.cvtColor(rgb, rgb, Imgproc.COLOR_RGB2GRAY);
//        rgb.setTo(new Scalar(0), finalMask);
//        Imgproc.adaptiveThreshold(rgb, rgb, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY_INV, 11, 2);

        // TODO contours

        return whiteMask;
    }
}
