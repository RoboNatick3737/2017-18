package org.firstinspires.ftc.teamcode.vision;

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
public class MaskGenerator2
{
    /**
     * Uses linear functions of a combination of other channels to filter other channels.
     *
     * |     \  /<-upper / <-lower value bound
     * | \     /        /
     * |  \   /\       /
     * |   \ /  \     /
     * |----/--------/------------- <- upper hue bound
     * |   / \****\ / <-- this area in *'s is what the algorithm would select
     * |--/--------/--------------- <- lower hue bound
     * | /     \  / \ <- upper saturation bound
     * |/        /
     * |___________________________
     *
     * Example is you want hue to be between .1 * value + 57 and -.1 * value + 255
     * You'd call commonAreaFilter(raw, result, HSV.HUE,
     *      new HSVMaskBound(HSV.VALUE, new LinearFunction(.1, 57)),
     *      new HSVMaskBound(HSV.VALUE, new LinearFunction(-.1, 255)));
     *
     * @param input The three-channel mat to filter (typically HSV).
     * @param output A mat of ones to which the filters will be applied.
     * @param channelBounds Linear bounds which describes the mask which will be selected.
     */
    public void commonAreaFilter(Mat input, Mat output, HSVMaskBound... channelBounds)
    {
        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(input, channels);

        output.setTo(new Scalar(1)); // Reset output

        // Apply the bounds and release them.
        for (int i = 0; i < 3; i++)
        {
            Mat mask = Mat.zeros(input.size(), Imgproc.THRESH_BINARY);
            channelBounds[i].apply(channels.get(i), mask);
            Core.bitwise_and(mask, output, output);
            mask.release();
        }
    }
}
