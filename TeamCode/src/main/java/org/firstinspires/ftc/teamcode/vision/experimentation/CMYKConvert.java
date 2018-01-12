package org.firstinspires.ftc.teamcode.vision.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.opencv.OpenCVJNIHooks;

@Autonomous(name="CS Convert", group= Constants.EXPERIMENTATION)
public class CMYKConvert extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{

    @Override
    public void onCameraViewStarted(int width, int height)
    {
    }

    @Override
    public void onCameraViewStopped()
    {
    }

    @Override
    public Mat onCameraFrame(Mat inputFrame)
    {
        Imgproc.cvtColor(inputFrame, inputFrame, Imgproc.COLOR_RGBA2RGB);

        Size original = inputFrame.size();

        Imgproc.resize(inputFrame, inputFrame, new Size(200, 100));

        for (int row = 0; row < inputFrame.rows(); row++)
        {
            for (int col = 0; col < inputFrame.cols(); col++)
            {
                double[] pixel = inputFrame.get(row, col);

                double r = pixel[0] / 255.0;
                double g = pixel[1] / 255.0;
                double b = pixel[2] / 255.0;

                double k = Math.min(Math.min(1 - r, 1 - g), 1 - b);

                double c = (1 - r - k) / (1 - k) * 255.0;

                inputFrame.put(row, col, 0, 0, c);
            }
        }

        Imgproc.resize(inputFrame, inputFrame, original);

        return inputFrame;
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
