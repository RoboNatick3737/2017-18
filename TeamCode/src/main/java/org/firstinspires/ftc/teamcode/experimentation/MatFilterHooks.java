package org.firstinspires.ftc.teamcode.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

import hankextensions.RobotCore;
import hankextensions.vision.opencv.OpenCVCam;
import visionanalysis.OpenCVJNIHooks;

@Autonomous(name="Mat filter — Hooks", group= Constants.EXPERIMENTATION)
public class MatFilterHooks extends RobotCore implements CameraBridgeViewBase.CvCameraViewListener
{
    private OpenCVCam openCVCam;

    private ProcessConsole cameraProcessConsole;

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


    /////// Analysis ///////

    private Size originalResolution;
    private Size analysisResolution;

    // Not constant mats
    private Mat luminanceFix, resultingLower, resultingUpper, blueMask, whiteMask;

    // A LinkedList into which channels can be split.
    private LinkedList<Mat> channels;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        originalResolution = new Size(width, height);
        analysisResolution = new Size(width, height);

        // init non-constant mats.
        luminanceFix = new Mat(analysisResolution, CvType.CV_8UC1);
        resultingLower = new Mat(analysisResolution, CvType.CV_8UC1);
        resultingUpper = new Mat(analysisResolution, CvType.CV_8UC1);
        blueMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        whiteMask = new Mat(analysisResolution, Imgproc.THRESH_BINARY);

        // Init channels list
        channels = new LinkedList<>();
    }

    @Override
    public void onCameraViewStopped()
    {
        resultingLower.release();
        resultingUpper.release();
        luminanceFix.release();
        whiteMask.release();
        blueMask.release();

        channels = null;
    }

    private void fixMatLuminance(Mat raw)
    {
        // Fix the lighting contrast that results from using different fields.
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2YCrCb);
        Core.split(raw, channels);
        Imgproc.equalizeHist(channels.get(0), channels.get(0));
        Core.merge(channels, raw);
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_YCrCb2RGB);
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        // Set low resolution for analysis (to speed this up)
        Imgproc.resize(raw, raw, analysisResolution);

        // Make colors appear sharper.
        fixMatLuminance(raw);

        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Analyze frame in HSV
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HSV);

        // Determine the upper and lower mat bounds for blue.
        Core.split(raw, channels);
        Mat hueChannel = channels.get(0);
        Mat saturationChannel = channels.get(1);
        Mat valueChannel = channels.get(2);

        // Adaptive value threshold for blue
        Core.subtract(valueChannel, new Scalar(55), valueChannel); // luminance - 55
        Core.multiply(valueChannel, new Scalar(-.1), luminanceFix); // -.1 * (luminance - 55)
        Core.add(luminanceFix, new Scalar(75), resultingLower); // blue hue > 75 + -.1 * (luminance - 55)
        Core.add(luminanceFix, new Scalar(135), resultingUpper); // blue hue < 135 + -.1 * (luminance - 55)
        OpenCVJNIHooks.inRangeBetweenMats(hueChannel, resultingLower, resultingUpper, blueMask); // resulting blue mask

        // Saturation has to be less than 59 for blue.
        Core.inRange(saturationChannel, new Scalar(59), new Scalar(255), saturationChannel); // for blue sat > 59

        // Get final mask (fits both value and saturation criteria.
        Core.bitwise_and(saturationChannel, blueMask, blueMask);

        // Get the white mask.
        Core.inRange(raw, new Scalar(0, 0, 49), new Scalar(255, 59, 255), whiteMask);

        // Display the results in the display mat.
        raw.setTo(new Scalar(0, 0, 0));
        raw.setTo(new Scalar(255, 255, 255), whiteMask);
        raw.setTo(new Scalar(100, 255, 255), blueMask);

        // Resize the image to the original size.
        Imgproc.resize(raw, raw, originalResolution);
        return raw;
    }
}
