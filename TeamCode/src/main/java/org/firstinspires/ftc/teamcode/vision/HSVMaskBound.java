package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.structs.LinearFunction;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import hankextensions.vision.opencv.OpenCVJNIHooks;

public class HSVMaskBound
{
    public final LinearFunction lowerBound, upperBound;

    public HSVMaskBound(LinearFunction lowerBound, LinearFunction upperBound)
    {
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
    }

    /**
     * Generates the linear mats based on the function provided.
     *
     * @param mat Mat to apply generation to
     * @param func function in question
     */
    private void applyFunction(Mat mat, LinearFunction func)
    {
        if (Math.abs(func.a) < .0001) // if it's zero just return a mat of one value.
        {
            Core.multiply(mat, new Scalar(func.a), mat);
            Core.add(mat, new Scalar(func.b), mat);
        } else
            mat.setTo(new Scalar(func.b));
    }

    public void apply(Mat input, Mat output)
    {
        Mat lower = input.clone();
        Mat upper = input.clone();

        applyFunction(lower, lowerBound);
        applyFunction(upper, upperBound);

        OpenCVJNIHooks.inRangeBetweenMats(input, lower, upper, output);

        lower.release();
        upper.release();
    }
}
