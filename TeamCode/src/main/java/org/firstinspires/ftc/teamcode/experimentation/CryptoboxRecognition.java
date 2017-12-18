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
    public Mat onCameraFrame(Mat raw)
    {
        // Fix the lighting contrast that results from using different fields.
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2YCrCb);
        Core.split(raw, channels);
        Imgproc.equalizeHist(channels.get(0), channels.get(0));
        Core.merge(channels, raw);
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_YCrCb2RGB);

        // Get blue mask.
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HSV);
        Core.inRange(raw, new Scalar(40, 0, 0), new Scalar(150, 255, 255), blueMask);
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_HSV2RGB);

        // Dilate the blue mask so we can find contours within it.
        Imgproc.dilate(blueMask, blueMask, Mat.ones(1, 50, CvType.CV_32F));

        // Invert the mask and neutralize non-included pixels in raw after making it grayscale.
//        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_BGR2GRAY);
        Core.bitwise_not(blueMask, blueMask);
        raw.setTo(new Scalar(0), blueMask);

        // Adaptive threshold the raw image.
//        Imgproc.adaptiveThreshold(raw, raw, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, 11, 2);
//        Imgproc.equalizeHist(raw, raw);

        return raw;
    }
}
