package org.firstinspires.ftc.teamcode.vision.colormasks;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.opencv.OpenCVJNIHooks;

@Autonomous(name="CMYK Red/Blue", group= Constants.VISION_TESTING)
public class CMYKFilter extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this);

        while (true)
            flow.yield();
    }

    private Mat blueMask, redMask;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        Size size = new Size(width, height);
        blueMask = new Mat(size, Imgproc.THRESH_BINARY);
        redMask = new Mat(size, Imgproc.THRESH_BINARY);
    }

    @Override
    public void onCameraViewStopped()
    {
        blueMask.release();
        redMask.release();
    }

    @Override
    public Mat onCameraFrame(Mat inputFrame)
    {
        Imgproc.cvtColor(inputFrame, inputFrame, Imgproc.COLOR_RGBA2RGB);

        OpenCVJNIHooks.cmykConvert(inputFrame);

        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(inputFrame, channels);

        // Emphasize contrast and decide filters.
        Imgproc.equalizeHist(channels.get(0), channels.get(0));
        Imgproc.threshold(channels.get(0), blueMask, 200, 255, Imgproc.THRESH_BINARY);
        Imgproc.equalizeHist(channels.get(1), channels.get(1));
        Imgproc.threshold(channels.get(1), redMask, 200, 255, Imgproc.THRESH_BINARY);

        // Apply filters.
        inputFrame.setTo(new Scalar(0, 0, 0));
        inputFrame.setTo(new Scalar(255, 0, 0), redMask);
        inputFrame.setTo(new Scalar(0, 0, 255), blueMask);

        return inputFrame;
    }
}
