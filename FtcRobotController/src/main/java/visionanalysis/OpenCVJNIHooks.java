package visionanalysis;

import org.opencv.core.Mat;

/**
 * A set of hooks to the JNI C++ api instead of the Java one in order to use native methods.
 */
public class OpenCVJNIHooks
{
    public static void inRangeBetweenMats(Mat toFilter, Mat lower, Mat upper, Mat dest)
    {
        inRangeBetweenMats(toFilter.getNativeObjAddr(), lower.getNativeObjAddr(), upper.getNativeObjAddr(), dest.getNativeObjAddr());
    }

    private static native void inRangeBetweenMats(long toFilterAddress, long lowerAddress, long upperAddress, long destAddress);
}
