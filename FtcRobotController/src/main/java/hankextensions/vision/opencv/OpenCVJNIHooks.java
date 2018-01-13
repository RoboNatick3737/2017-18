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

    /**
     * Gets the mask which results from using two mats as the upper and lower bounds instead
     * of the scalar.
     *
     * @param toFilter The mat which will be filtered
     * @param lower    The lower bound
     * @param upper    The upper bound
     * @param dest     Where the resulting mask will be stored
     */
    public static void inRangeBetweenMats(Mat toFilter, Mat lower, Mat upper, Mat dest)
    {
        inRangeBetweenMatsNative(toFilter.getNativeObjAddr(), lower.getNativeObjAddr(), upper.getNativeObjAddr(), dest.getNativeObjAddr());
    }
    private static native void inRangeBetweenMatsNative(long toFilterAddress, long lowerAddress, long upperAddress, long destAddress);

    /**
     * Converts a given RGB mat to CYMK (additive to subtractive color mixing.
     *
     * @param toConvert The mat which will be converted.
     */
    public static void cmykConvert(Mat toConvert)
    {
        cmykConvert(toConvert.getNativeObjAddr());
    }
    private static native void cmykConvert(long toConvertAddress);

    /**
     * Once Java's done the simple filtering by ensuring that each positive has some blue and white,
     * this does the more complex filtering (checking each region meets specific criteria, etc.
     *
     * @param toAnalyze  The mat to analyze.
     * @param array      The current array of positive hits (to be further filtered).
     */
    public static void deepCryptoboxAnalysis(Mat toAnalyze, Mat primaryMask, Mat whiteMask, boolean[] array)
    {
        deepCryptoboxAnalysis(toAnalyze.getNativeObjAddr(), primaryMask.getNativeObjAddr(), whiteMask.getNativeObjAddr(), array);
    }
    private static native void deepCryptoboxAnalysis(long toAnalyzeAddress, long primaryMaskAddress, long whiteMaskAddress, boolean[] array);
}
