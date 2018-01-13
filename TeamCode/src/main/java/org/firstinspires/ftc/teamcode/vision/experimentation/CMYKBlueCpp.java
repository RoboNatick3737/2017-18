package org.firstinspires.ftc.teamcode.vision.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.opencv.OpenCVJNIHooks;

@Autonomous(name="CMYK Blue — C++", group= Constants.VISION_TESTING)
public class CMYKBlueCpp extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    @Override
    public void onCameraViewStarted(int width, int height)
    {}

    @Override
    public void onCameraViewStopped()
    {}

    @Override
    public Mat onCameraFrame(Mat inputFrame)
    {
        Imgproc.cvtColor(inputFrame, inputFrame, Imgproc.COLOR_RGBA2RGB);

        Size original = inputFrame.size();

        Imgproc.resize(inputFrame, inputFrame, new Size(200, 100));

        OpenCVJNIHooks.cmykConvert(inputFrame);

        Imgproc.resize(inputFrame, inputFrame, original);

        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(inputFrame, channels);

        return channels.get(0);
    }

    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this, false);

        ProcessConsole console = log.newProcessConsole("Console");

        while (true)
        {
            flow.yield();
        }
    }
}
