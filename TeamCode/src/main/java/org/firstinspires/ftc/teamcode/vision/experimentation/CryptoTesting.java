package org.firstinspires.ftc.teamcode.vision.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Crypto Testing", group= Constants.EXPERIMENTATION)
public class CryptoTesting extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    private double blueMin = 200, whiteMin = 200;

    @Override
    public void onCameraViewStarted(int width, int height) {
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(Mat inputFrame)
    {
        // Convert to a great color space for detecting blue (Cb).
        Imgproc.cvtColor(inputFrame, inputFrame, Imgproc.COLOR_RGB2YCrCb);
        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(inputFrame, channels);

        // Get blue mask
        Mat blue = channels.get(2);
        Imgproc.equalizeHist(blue, blue); // Contrast
        Imgproc.threshold(blue, blue, blueMin, 255, Imgproc.THRESH_BINARY);

        // Get white mask
        Mat white = channels.get(0);
        Imgproc.equalizeHist(white, white); // Contrast
        Imgproc.threshold(white, white, whiteMin, 255, Imgproc.THRESH_BINARY);

        Mat cryptobox = new Mat();
        Core.bitwise_or(white, blue, cryptobox);

        inputFrame.setTo(new Scalar(0, 0, 255), blue);
        inputFrame.setTo(new Scalar(255, 255, 255), white);

        for (int i = 0; i < inputFrame.cols(); i++)
        {
            Mat cryptoCol = cryptobox.col(i);
            Mat blueCol = blue.col(i);
            Mat whiteCol = white.col(i);

            if (Core.countNonZero(cryptoCol) > .8 * inputFrame.rows() &&
                    Core.countNonZero(blueCol) > 3.5 * Core.countNonZero(whiteCol))
                inputFrame.col(i).setTo(new Scalar(127, 255, 0));
        }

        cryptobox.release();

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
                blueMin += .0001;
            else if (gamepad1.dpad_down)
                blueMin -= .0001;

            if (gamepad1.dpad_right)
                whiteMin += .0001;
            else if (gamepad1.dpad_left)
                whiteMin -= .0001;

            console.write("Blue min = " + blueMin, "White min = " + whiteMin);

            flow.yield();
        }
    }
}
