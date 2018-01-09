package org.firstinspires.ftc.teamcode.vision.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Contrast Grayscale", group= Constants.EXPERIMENTATION)
public class ContrastGrayscale extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    private double min = 50, max = 150;

    @Override
    public void onCameraViewStarted(int width, int height) {
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(Mat inputFrame)
    {
        Imgproc.cvtColor(inputFrame, inputFrame, Imgproc.COLOR_RGBA2GRAY);
        Imgproc.equalizeHist(inputFrame, inputFrame);

        Imgproc.threshold(inputFrame, inputFrame, min, max, Imgproc.THRESH_BINARY);

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
            if (gamepad1.dpad_up)
            {
                min += .001;
            }
            else if (gamepad1.dpad_down)
            {
                min -= .001;
            }

            if (gamepad1.dpad_left)
            {
                max -= .001;
            }
            else if (gamepad1.dpad_right)
            {
                max += .001;
            }

            console.write("Min = " + min, "Max = " + max);

            flow.yield();
        }
    }
}
