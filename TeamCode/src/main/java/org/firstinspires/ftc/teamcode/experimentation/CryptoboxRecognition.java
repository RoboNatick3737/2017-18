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

    private Mat resize, lContours;

    private ProcessConsole cameraProcessConsole;

    private int width, height;

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
        this.width = width;
        this.height = height;

        resize = new Mat();
        lContours = new Mat();
    }

    @Override
    public void onCameraViewStopped()
    {
        resize.release();
        lContours.release();
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
//        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGBA2RGB);
//        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HLS);

        // Extract blue region.
//        Core.inRange(raw, new Scalar(200, 0, 0), new Scalar(255, 255, 255), raw);
//
//        Mat structure = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,12));
//        Imgproc.morphologyEx(blue,blue,Imgproc.MORPH_CLOSE, structure);
//        Imgproc.erode(blue, blue, Mat.ones(15, 3, CvType.CV_32F));

//        Imgproc.resize(lContours, blue, new Size(width, height));

        return raw;
    }
}
