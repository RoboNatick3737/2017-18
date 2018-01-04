package org.firstinspires.ftc.teamcode.vision;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Originally developed by Alex from DogeCV, I'm modifying a bit for my own purposes: https://github.com/GTHSRobotics/DogeCV
 */
public class JewelDetector2 implements CameraBridgeViewBase.CvCameraViewListener
{
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
    private Mat blueMask, redMask, displayMat;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        originalResolution = new Size(width, height);
        analysisResolution = new Size(width, height);
        maskGenerator = new MaskGenerator(analysisResolution);

        // init non-constant mats.
        blueMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        redMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        displayMat = new Mat(analysisResolution, CvType.CV_8UC1);
    }

    @Override
    public void onCameraViewStopped()
    {
        blueMask.release();
        redMask.release();
        displayMat.release();
        maskGenerator.releaseMats();
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        // Set low resolution for analysis (to speed this up)
        Imgproc.resize(raw, raw, analysisResolution);

        // Make colors appear sharper.
        maskGenerator.fixMatLuminance(raw);

        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Analyze frame in HSV
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HSV);

        // Get the blue mask with adaptive hsv.
        maskGenerator.adaptiveHSV(raw, 55, 75, -.1, 135, -.1, 59, 255, blueMask);

        // Get the red mask with adaptive hsv.
        maskGenerator.adaptiveHSV(raw, 55, 0, 0, 20, .1, 59, 255, redMask);

        // Set values
        displayMat.setTo(new Scalar(0)); // clear existent values
        displayMat.setTo(new Scalar(50), blueMask);
        displayMat.setTo(new Scalar(255), redMask);

        // Resize to the original (crashes otherwise)
        Imgproc.resize(displayMat, displayMat, originalResolution);
        return displayMat;
    }
}