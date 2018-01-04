package org.firstinspires.ftc.teamcode.opmodes.experimentation.vision;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Red and Blue 1", group= Constants.EXPERIMENTATION)
public class RednBlue extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    private OpenCVCam openCVCam;

    private ProcessConsole cameraProcessConsole;

    @Override
    protected void onRun() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        openCVCam.start(this);

        cameraProcessConsole = log.newProcessConsole("Camera Process Console");

        waitForStart();

        while (true)
        {
            flow.yield();
        }
    }

    private Mat redMask, blueMask;

    @Override
    public void onCameraViewStarted(int width, int height) {
        redMask = new Mat();
        blueMask = new Mat();
    }

    @Override
    public void onCameraViewStopped() {
        redMask.release();
        blueMask.release();
    }

    @Override
    public Mat onCameraFrame(Mat inputFrame) {
        getRedMask(inputFrame, redMask);
        getRedMask(inputFrame, blueMask);

        inputFrame.setTo(new Scalar(0, 0, 0));
        inputFrame.setTo(new Scalar(255, 0, 0), redMask);
        inputFrame.setTo(new Scalar(0, 0, 255), blueMask);

        return inputFrame;
    }

    private void getRedMask(Mat input, Mat dest)
    {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab);
        Imgproc.GaussianBlur(input,input,new Size(3,3),0);
        List<Mat> channels = new ArrayList<Mat>();
        Core.split(input, channels);
        Imgproc.threshold(channels.get(1), dest, 164.0, 255, Imgproc.THRESH_BINARY);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_Lab2RGB);
    }

    private void getBlueMask(Mat input, Mat dest)
    {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
        Imgproc.GaussianBlur(input,input,new Size(3,3),0);
        List<Mat> channels = new ArrayList<Mat>();
        Core.split(input, channels);
        Imgproc.threshold(channels.get(1), dest, 145.0, 255, Imgproc.THRESH_BINARY);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_YUV2RGB);
    }
}
