package org.firstinspires.ftc.teamcode.vision.filteringutilities.commonareafilter;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

/**
 * A utility class for my detectors which uses JNI hooks and such to improve the resulting masks.
 */
public class ThreeChannelProportionalFilter
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
     *      new LinearChannelBound(HSV.VALUE, new Linear(.1, 57)),
     *      new LinearChannelBound(HSV.VALUE, new Linear(-.1, 255)));
     *
     * @param input The three-channel mat to filter (typically HSV).
     * @param output A mat of ones to which the filters will be applied.
     * @param channel1Bound Linea bounds for channel 1 which describes the mask which will be selected.
     */
    public static void commonAreaFilter(Mat input, Mat output, LinearChannelBound channel1Bound, LinearChannelBound channel2Bound, LinearChannelBound channel3Bound)
    {
        LinearChannelBound[] channelBounds = {channel1Bound, channel2Bound, channel3Bound};

        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(input, channels);

        output.setTo(new Scalar(1)); // Reset output

        // Apply the bounds and release them.
        for (int i = 0; i < 3; i++)
        {
            if (channelBounds[i] == null)
                continue;

            Mat mask = Mat.zeros(input.size(), Imgproc.THRESH_BINARY);

            switch (i)
            {
                case 0:
                    channelBounds[i].filter(channels.get(0), channels.get(1), channels.get(2), mask);
                    break;

                case 1:
                    channelBounds[i].filter(channels.get(1), channels.get(0), channels.get(2), mask);
                    break;

                case 2:
                    channelBounds[i].filter(channels.get(2), channels.get(0), channels.get(1), mask);
                    break;
            }
            Core.bitwise_and(mask, output, output);
            mask.release();
        }
    }
}
