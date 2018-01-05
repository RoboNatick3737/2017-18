package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.structs.LinearFunction;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

import hankextensions.vision.opencv.OpenCVJNIHooks;

/**
 * A utility class for my detectors which uses JNI hooks and such to improve the resulting masks.
 */
public class MaskGenerator
{
    // Pre-initialized mats.
    private Mat minimumValueGradient, maximumValueGradient, resultingLower, resultingUpper, mask;

    // A LinkedList into which channels can be split.
    private LinkedList<Mat> channels;

    /**
     * Instantiate all required mats, etc.
     */
    public MaskGenerator(Size analysisResolution)
    {
        // init non-constant mats.
        minimumValueGradient = new Mat(analysisResolution, CvType.CV_8UC1);
        maximumValueGradient = new Mat(analysisResolution, CvType.CV_8UC1);
        resultingLower = new Mat(analysisResolution, CvType.CV_8UC1);
        resultingUpper = new Mat(analysisResolution, CvType.CV_8UC1);
        mask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image

        // Init channels list
        channels = new LinkedList<>();
    }

    /**
     * Releases all mats in use.
     */
    public void releaseMats()
    {
        resultingLower.release();
        resultingUpper.release();
        minimumValueGradient.release();
        maximumValueGradient.release();
        mask.release();

        channels = null;
    }

    /**
     * Uses equalizeHist to equalize the luminance channel (in order to view things approximately
     * equally in poor light conditions).
     *
     * @param raw The mat to fix (duh)
     */
    public void fixMatLuminance(Mat raw)
    {
        // Fix the lighting contrast that results from using different fields.
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2YCrCb);
        Core.split(raw, channels);
        Imgproc.equalizeHist(channels.get(0), channels.get(0));
        Core.merge(channels, raw);
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_YCrCb2RGB);
    }

    /**
     * Adaptively filters color based on luminance.
     *
     * @param hsv The hsv mat
     * @param minHue The minimum hue
     * @param minValSlope minimum mat = min hue + minValSlope * luminance
     * @param maxHue The maximum hue
     * @param maxValSlope maximum mat = max hue + maxValSlope * luminance
     * @param minSat the minimum mat saturation.
     * @param dest The destination mat
     */
    public void adaptiveHSV(final Mat hsv, final double valOffset, final double minHue, final double minValSlope, final double maxHue, final double maxValSlope, final double minSat, final double maxSat, final Mat dest)
    {
        // Determine the upper and lower mat bounds for blue.
        Core.split(hsv, channels);

        // Use overload with channels.
        adaptiveHSV(channels, valOffset, minHue, minValSlope, maxHue, maxValSlope, minSat, maxSat, dest);
    }

    /**
     * Adaptively filters color based on luminance.
     *
     * @param channels The hsv mat in channels
     * @param minHue The minimum hue
     * @param minValSlope minimum mat = min hue + minValSlope * luminance
     * @param maxHue The maximum hue
     * @param maxValSlope maximum mat = max hue + maxValSlope * luminance
     * @param minSat the minimum mat saturation.
     * @param dest The destination mat
     */
    public void adaptiveHSV(final LinkedList<Mat> channels, final double valOffset, final double minHue, final double minValSlope, final double maxHue, final double maxValSlope, final double minSat, final double maxSat, final Mat dest)
    {
        // Make some pointers.
        Mat hueChannel = channels.get(0), saturationChannel = channels.get(1), valueChannel = channels.get(2);

        // Adaptive value threshold for the mat with JNI hook
        Core.subtract(valueChannel, new Scalar(valOffset), valueChannel); // luminance - 55

        Core.multiply(valueChannel, new Scalar(minValSlope), minimumValueGradient); // -.1 * (luminance - 55)
        Core.add(minimumValueGradient, new Scalar(minHue), resultingLower); // blue hue > 75 + -.1 * (luminance - 55)

        // Don't make another luminance fix mat if unnecessary.
        if (Math.abs(minValSlope - maxValSlope) > .1) // if difference is substantial (double equality)
        {
            Core.multiply(valueChannel, new Scalar(maxValSlope), maximumValueGradient); // -.1 * (luminance - 55)
            Core.add(maximumValueGradient, new Scalar(maxHue), resultingUpper); // blue hue < 135 + -.1 * (luminance - 55)
        }
        else
        {
            Core.add(minimumValueGradient, new Scalar(maxHue), resultingUpper); // blue hue < 135 + -.1 * (luminance - 55)
        }

        // Use the JNI to find the area between the two mats.
        OpenCVJNIHooks.inRangeBetweenMats(hueChannel, resultingLower, resultingUpper, mask); // resulting blue mask

        // Saturation has to be less than 59 for blue.
        Core.inRange(saturationChannel, new Scalar(minSat), new Scalar(maxSat), saturationChannel); // for blue sat > 59

        // Get final mask (fits both value and saturation criteria).
        Core.bitwise_and(saturationChannel, mask, dest);
    }
}
