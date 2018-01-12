package hankextensions.vision.opencv;

import org.opencv.core.Mat;

/**
 * A set of hooks to the JNI C++ api instead of the Java one in order to use native methods.
 */
public class OpenCVJNIHooks
{
    // This HAS to be loaded or the app will crash upon attempting to call a native method.
    static
    {
        System.loadLibrary("native-opencv");
    }

    public static void inRangeBetweenMats(Mat toFilter, Mat lower, Mat upper, Mat dest)
    {
        inRangeBetweenMatsNative(toFilter.getNativeObjAddr(), lower.getNativeObjAddr(), upper.getNativeObjAddr(), dest.getNativeObjAddr());
    }

    private static native void inRangeBetweenMatsNative(long toFilterAddress, long lowerAddress, long upperAddress, long destAddress);

    public static void cmykConvert(Mat toConvert)
    {
        cmykConvert(toConvert.getNativeObjAddr());
    }

    private static native void cmykConvert(long toConvertAddress);
}
