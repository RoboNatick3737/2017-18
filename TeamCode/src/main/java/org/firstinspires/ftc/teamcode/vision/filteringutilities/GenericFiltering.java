package org.firstinspires.ftc.teamcode.vision.filteringutilities;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Generic Filtering", group= Constants.EXPERIMENTATION)
public class GenericFiltering extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    private static double BLUE_MIN = 220, RED_MIN = 200, WHITE_MIN = 160;

    public static void blueFilter(Mat raw, Mat blueMask)
    {
        // Convert to a great color space for detecting blue (Cb).
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2YCrCb);
        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(raw, channels);

        // Get blue mask
        Mat blue = channels.get(2);
        Imgproc.equalizeHist(blue, blue); // Contrast
        Imgproc.threshold(blue, blueMask, BLUE_MIN, 255, Imgproc.THRESH_BINARY);
    }

    public static void redFilter(Mat raw, Mat redMask)
    {
        // Convert to a great color space for detecting red (Cr).
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2YCrCb);
        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(raw, channels);

        // Get red mask
        Mat red = channels.get(1);
        Imgproc.equalizeHist(red, red); // Contrast
        Imgproc.threshold(red, redMask, RED_MIN, 255, Imgproc.THRESH_BINARY);
    }

    public static void whiteFilter(Mat raw, Mat whiteMask)
    {
        // Convert to a great color space for detecting white (Y).
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2YCrCb);
        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(raw, channels);

        // Get blue mask
        Mat white = channels.get(0);
        Imgproc.equalizeHist(white, white); // Contrast
        Imgproc.threshold(white, whiteMask, WHITE_MIN, 255, Imgproc.THRESH_BINARY);
    }

    private enum PrimaryColor {RED, BLUE, WHITE}
    private PrimaryColor currentlyAdjusting = PrimaryColor.RED;
    private Mat red, blue, white;

    @Override
    public void onCameraViewStarted(int width, int height) {
        red = new Mat();
        blue = new Mat();
        white = new Mat();
    }

    @Override
    public void onCameraViewStopped() {
        red.release();
        blue.release();
        white.release();
    }

    @Override
    public Mat onCameraFrame(Mat inputFrame)
    {
        blueFilter(inputFrame, blue);
        redFilter(inputFrame, red);
        whiteFilter(inputFrame, white);

        inputFrame.setTo(new Scalar(0, 0, 0));
        inputFrame.setTo(new Scalar(255, 0, 0), red);
        inputFrame.setTo(new Scalar(0, 0, 255), blue);
        inputFrame.setTo(new Scalar(255, 255, 255), white);

        return inputFrame;
    }

    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this, true);

        ProcessConsole console = log.newProcessConsole("Console");

        while (true)
        {
            if (gamepad1.y)
                currentlyAdjusting = PrimaryColor.RED;
            else if (gamepad1.a)
                currentlyAdjusting = PrimaryColor.BLUE;
            else if (gamepad1.b)
                currentlyAdjusting = PrimaryColor.WHITE;

            double adjustment;
            if (gamepad1.dpad_up)
                adjustment = .0001;
            else if (gamepad1.dpad_down)
                adjustment = -.0001;
            else
                adjustment = 0;

            switch (currentlyAdjusting)
            {
                case BLUE:
                    BLUE_MIN += adjustment;
                    break;

                case RED:
                    RED_MIN += adjustment;
                    break;

                case WHITE:
                    WHITE_MIN += adjustment;
                    break;
            }

            console.write("Blue min " + BLUE_MIN, "Red min " + RED_MIN, "White min " + WHITE_MIN);

            flow.yield();
        }
    }
}
