package org.firstinspires.ftc.teamcode.vision.filteringutilities.commonareafilter;

import org.firstinspires.ftc.teamcode.vision.filteringutilities.LinearFunctionBounds;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import hankextensions.vision.opencv.OpenCVJNIHooks;

/**
 * This works as follows: this represents one channel of a 3-channel mat.  Two other channels
 * have functions overlaid on the first one.
 */
public class LinearChannelBound
{
    public final LinearFunctionBounds bound1, bound2;

    public LinearChannelBound(LinearFunctionBounds otherChannel1Bounds, LinearFunctionBounds otherChannel2Bounds)
    {
        this.bound1 = otherChannel1Bounds;
        this.bound2 = otherChannel2Bounds;
    }

    private void applyFunctionBounds(Mat toward, Mat against, LinearFunctionBounds linearBounds, Mat output)
    {
        boolean noSlopeForA = Math.abs(linearBounds.lower.a) < .001, noSlopeForB = Math.abs(linearBounds.upper.a) < .001;

        if (noSlopeForA && noSlopeForB)
        {
            Core.inRange(toward, new Scalar(linearBounds.lower.b), new Scalar(linearBounds.upper.b), toward);
            return;
        }

        Mat upper = against.clone(), lower = against.clone();

        // Init mat multiplication and stuff
        if (!noSlopeForA)
        {
            Core.multiply(lower, new Scalar(linearBounds.lower.a), lower);
            Core.add(lower, new Scalar(linearBounds.lower.b), lower);
        } else
            lower.setTo(new Scalar(linearBounds.lower.b));

        if (!noSlopeForA)
        {
            Core.multiply(upper, new Scalar(linearBounds.upper.a), upper);
            Core.add(upper, new Scalar(linearBounds.upper.b), upper);
        } else
            upper.setTo(new Scalar(linearBounds.upper.b));

        // Do the range
        OpenCVJNIHooks.inRangeBetweenMats(toward, lower, upper, output);

        // Release upper and lower
        upper.release();
        lower.release();
    }

    public void filter(Mat toModify, Mat otherChan1, Mat otherChan2, Mat binaryMaskOutput)
    {
        Mat firstResult = Mat.zeros(toModify.size(), Imgproc.THRESH_BINARY);
        applyFunctionBounds(toModify, otherChan1, bound1, firstResult);

        Mat secondResult = Mat.zeros(toModify.size(), Imgproc.THRESH_BINARY);
        applyFunctionBounds(toModify, otherChan2, bound2, secondResult);

        Core.bitwise_and(firstResult, secondResult, binaryMaskOutput);

        firstResult.release();
        secondResult.release();
    }
}
