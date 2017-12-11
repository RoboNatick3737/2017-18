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
    private Mat blueMask, lContours;

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
    }

    @Override
    public void onCameraViewStopped()
    {
        blueMask.release();
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HLS);

        Core.split(raw, channels);

        // Blur the hue (not the luminance).
        Imgproc.blur(channels.get(0), channels.get(0), new Size(3, 3));

        // Set generous bounds for a vaguely blue hue before doing the adaptive threshold thing.
        Core.inRange(channels.get(0), new Scalar(40), new Scalar(150), blueMask);

        // Invert the blue mask.
        Core.bitwise_not(blueMask, blueMask);

        // Set all pixels in the inverted mask to zero.
        channels.get(1).setTo(new Scalar(0), blueMask);

        // Do the adaptive threshold bit
        Imgproc.adaptiveThreshold(channels.get(1), channels.get(1), 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY_INV, 11, 2);

        // TODO contours

        return channels.get(1);
    }
}
