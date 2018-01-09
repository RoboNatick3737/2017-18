package org.firstinspires.ftc.teamcode.vision.analysis;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
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

/**
 * Originally developed by Alex from DogeCV, I'm modifying a bit for my own purposes: https://github.com/GTHSRobotics/DogeCV
 */
public class JewelDetector2 extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this, true);

        ProcessConsole console = LoggingBase.instance.newProcessConsole("Jewel Detector 2");

        while (true)
        {
            console.write("Order is " + getCurrentOrder().toString());
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

    // A mask generator (blue and red adaptively)
    private MaskGenerator maskGenerator;

    // Mat sizes which constitute analysis vs. the size of the frame we were originally passed.
    private Size originalResolution;
    private Size analysisResolution;

    // Pre-initialized mats.
    private Mat blueMask, redMask;
    private boolean[] jewelColumns;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        originalResolution = new Size(width, height);
        analysisResolution = new Size(width, height);
        maskGenerator = new MaskGenerator(analysisResolution);

        // init non-constant mats.
        blueMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        redMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
    }

    @Override
    public void onCameraViewStopped()
    {
        blueMask.release();
        redMask.release();
        maskGenerator.releaseMats();
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        // Set low resolution for analysis (to speed this up)
        Imgproc.resize(raw, raw, analysisResolution);

        // Make colors appear sharper.
        AdditionalFilteringUtilities.fixMatLuminance(raw);

        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Analyze frame in HSV
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2YCrCb);

        // Get blue and red mask.
        

        // convert back to rgb
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_YCrCb2RGB);

        // Set values
        raw.setTo(new Scalar(0, 0, 255), blueMask);
        raw.setTo(new Scalar(255, 0, 0), redMask);

        // Determine which rows constitute the area where the glyphs are (remember that the phone is inverted upside down though).
        jewelColumns = new boolean[(int)(analysisResolution.width)];
        for (int i = 0; i < raw.cols(); i++)
        {
            int nonZeroBlue = Core.countNonZero(blueMask.col(i));
            int nonZeroRed = Core.countNonZero(redMask.col(i));

            if (nonZeroBlue > analysisResolution.height * .1 && nonZeroRed > analysisResolution.height * .1)
                jewelColumns[i] = true;
        }

        // Decide which one is the jewel location based on the largest mass of booleans.
        int longest = 0, index = 0, current = 0;
        for (int i = 0; i < jewelColumns.length; i++)
        {
            if (!jewelColumns[i])
            {
                if (current > longest)
                {
                    longest = current;
                    index = i - current;
                }

                current = 0;
            }
            else
                current++;
        }
        // Otherwise last column won't be counted.
        if (current > 0)
        {
            if (current > longest)
            {
                longest = current;
                index = jewelColumns.length - current - 1;
            }
        }

        // Make the chosen one visible.
        for (int i = 0; i < longest; i++)
            raw.col(index + i).setTo(new Scalar(255, 255, 255));

        // Decide whether the left or right side is =the blue jewel by the midpoint of the selected area: get 4 submats for blue for first half, blue for second half, red for first half, red for second half.
        int midpoint = (int)(blueMask.rows() / 2.0);
        Mat firstHalfBlueMat = blueMask.submat(0, midpoint, index, index + longest);
        int firstHalfBlue = Core.countNonZero(firstHalfBlueMat);
        firstHalfBlueMat.release();

        Mat secondHalfBlueMat = blueMask.submat(midpoint + 1, blueMask.rows(), index, index + longest);
        int secondHalfBlue = Core.countNonZero(secondHalfBlueMat);
        secondHalfBlueMat.release();

        Mat firstHalfRedMat = redMask.submat(0, midpoint, index, index + longest);
        int firstHalfRed = Core.countNonZero(firstHalfRedMat);
        firstHalfRedMat.release();

        Mat secondHalfRedMat = redMask.submat(midpoint, redMask.rows(), index, index + longest);
        int secondHalfRed = Core.countNonZero(secondHalfRedMat);
        secondHalfRedMat.release();

        if (firstHalfBlue > firstHalfRed && secondHalfRed > secondHalfBlue)
        {
            currentOrder = JewelOrder.BLUE_RED;
        }
        else if (firstHalfBlue < firstHalfRed && secondHalfRed < secondHalfBlue)
        {
            currentOrder = JewelOrder.RED_BLUE;
        }
        else
        {
            currentOrder = JewelOrder.UNKNOWN;
        }

        // Resize to the original (crashes otherwise)
        Imgproc.resize(raw, raw, originalResolution);
        return raw;
    }
}