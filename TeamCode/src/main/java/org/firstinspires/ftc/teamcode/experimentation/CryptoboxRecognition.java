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

    private int frameHeight, frameWidth;
    private double frameConversionHeight, frameConversionWidth;

    private Mat kernel;

    private ProcessConsole cameraProcessConsole;

    @Override
    protected void INITIALIZE()
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        openCVCam.start();

        openCVCam.setCameraFrameListener(this);
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
        frameHeight = height;
        frameWidth = width;

        frameConversionWidth = width / 740.0;
        frameConversionHeight = height / 360.0;

        kernel = Mat.ones((int)(frameConversionHeight * 15),(int)(frameConversionWidth * 3),CvType.CV_32F);
    }

    @Override
    public void onCameraViewStopped()
    {
        kernel.release();
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        log.lines("Called!");

        ///// Get rid of high luminance pixels (working in HLS space) ////
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HLS);
        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(raw, channels);
        Mat mask = new Mat();
        Imgproc.threshold(channels.get(1), mask, 0, 240, Imgproc.THRESH_BINARY_INV + Imgproc.THRESH_OTSU);
        channels.get(1).setTo(new Scalar(100), mask);
        channels.get(2).setTo(new Scalar(255), mask);
        Core.merge(channels, raw);

        ////// Filter based on RGB ///////
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_HLS2RGB);
        Core.inRange(raw,
                new Scalar(0, 0, 170),
                new Scalar(52, 93, 255), raw);

        ////// Get rid of excessive noise ///////
        Imgproc.blur(raw, raw, new Size(2, 10));
        Imgproc.erode(raw,raw,kernel);
        Imgproc.dilate(raw,raw,kernel);
        Imgproc.threshold(raw, raw, 200, 255, Imgproc.THRESH_BINARY);

        // TODO filter contours

        return raw;
    }
}
