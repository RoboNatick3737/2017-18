package org.firstinspires.ftc.teamcode.vision.analysis;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.AdditionalFilteringUtilities;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.MaskGenerator;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Balance Plate Jewel Vision 2", group= Constants.EXPERIMENTATION)
public class BalancePlateJewelVision2 extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    private OpenCVCam openCVCam;

    private ProcessConsole cameraProcessConsole;

    @Override
    protected void onRun() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        openCVCam.start(this, true);

        cameraProcessConsole = log.newProcessConsole("Camera Process Console");

        waitForStart();

        while (true)
        {
            cameraProcessConsole.write("Current is " + getCurrentOrder().toString());
            flow.yield();
        }
    }

    // From left to right.
    public enum JewelOrder {
        RED_BLUE,
        BLUE_RED,
        UNKNOWN
    }
    // The results.
    private JewelOrder currentOrder = JewelOrder.UNKNOWN;
    public JewelOrder getCurrentOrder() {
        return currentOrder;
    }

    private MaskGenerator maskGenerator;
    private Mat blueMask, redMask;
    private Size analysisResolution;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        analysisResolution = new Size((int)(width * .07), (int)(height * .68));

        maskGenerator = new MaskGenerator(analysisResolution);

        // init non-constant mats.
        blueMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        redMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
    }

    @Override
    public void onCameraViewStopped() {
        maskGenerator.releaseMats();
        blueMask.release();
        redMask.release();
    }

    @Override
    public Mat onCameraFrame(Mat original)
    {
        // Draw onscreen to show where jewels should be.
        for (int i = (int)(original.rows() * .52); i < (int)(original.rows() * .6); i++)
            original.row(i).setTo(new Scalar(0, 0, 0));
        for (int i = (int)(original.cols() * .87); i < (int)(original.cols() * .93); i++)
            original.col(i).setTo(new Scalar(0, 0, 0));

        // Make red and blue appear as red and blue.
        Mat jewelSubmat = original.submat((int)(original.rows() * .22), (int)(original.rows() * .9), (int)(original.cols() * .93), original.cols());

        // Make colors appear sharper.
        AdditionalFilteringUtilities.fixMatLuminance(jewelSubmat);

        // Remove noise from image.
        Imgproc.blur(jewelSubmat, jewelSubmat, new Size(3, 3));

        // Analyze frame in HSV
        Imgproc.cvtColor(jewelSubmat, jewelSubmat, Imgproc.COLOR_RGB2HSV);

        // Get the blue mask with adaptive hsv.
        maskGenerator.adaptiveHSV(jewelSubmat, 55, 75, -.1, 135, -.1, 59, 255, blueMask);

        // Get the red mask with adaptive hsv.
        maskGenerator.adaptiveHSV(jewelSubmat, 55, 0, 0, 20, .1, 59, 255, redMask);

        // Decide whether the left or right side is =the blue jewel by the midpoint of the selected area: get 4 submats for blue for first half, blue for second half, red for first half, red for second half.
        int midpoint = (int)(blueMask.rows() / 2.0);
        Mat firstHalfBlueMat = blueMask.submat(0, midpoint, 0, blueMask.cols());
        int firstHalfBlue = Core.countNonZero(firstHalfBlueMat);
        firstHalfBlueMat.release();

        Mat secondHalfBlueMat = blueMask.submat(midpoint + 1, blueMask.rows(), 0, blueMask.cols());
        int secondHalfBlue = Core.countNonZero(secondHalfBlueMat);
        secondHalfBlueMat.release();

        Mat firstHalfRedMat = redMask.submat(0, midpoint, (int)(blueMask.cols() * .22), (int)(blueMask.cols() * .55));
        int firstHalfRed = Core.countNonZero(firstHalfRedMat);
        firstHalfRedMat.release();

        Mat secondHalfRedMat = redMask.submat(midpoint, redMask.rows(), (int)(blueMask.cols() * .6), (int)(blueMask.cols() * .93));
        int secondHalfRed = Core.countNonZero(secondHalfRedMat);
        secondHalfRedMat.release();

        if ((firstHalfBlue > secondHalfBlue) && (secondHalfRed > firstHalfRed))
        {
            currentOrder = JewelOrder.RED_BLUE;
        }
        else if ((secondHalfBlue > firstHalfBlue) && (firstHalfRed > secondHalfRed))
        {
            currentOrder = JewelOrder.BLUE_RED;
        }
        else
        {
            currentOrder = JewelOrder.UNKNOWN;
        }

//        jewelSubmat.setTo(new Scalar(0, 0, 255), blueMask);
//        jewelSubmat.setTo(new Scalar(255, 0, 0), redMask);

//        Imgproc.resize(jewelSubmat, original, original.size());

        jewelSubmat.release();

        return original;
    }
}
