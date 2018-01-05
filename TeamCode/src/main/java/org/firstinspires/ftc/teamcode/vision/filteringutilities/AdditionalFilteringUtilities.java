package org.firstinspires.ftc.teamcode.vision.filteringutilities;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

public class AdditionalFilteringUtilities
{
    /**
     * Uses equalizeHist to equalize the luminance channel (in order to view things approximately
     * equally in poor light conditions).
     *
     * @param raw The mat to fix (duh)
     */
    public static void fixMatLuminance(Mat raw)
    {
        // Fix the lighting contrast that results from using different fields.
        LinkedList<Mat> channels = new LinkedList<>();
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2YCrCb);
        Core.split(raw, channels);
        Imgproc.equalizeHist(channels.get(0), channels.get(0));
        Core.merge(channels, raw);
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_YCrCb2RGB);
    }
}
