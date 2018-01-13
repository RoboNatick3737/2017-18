#include <jni.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

extern "C"
{

/**
 * Randomly sprinkles some white pixels into a mat.
 *
 * @param matAddrGray  Mat address
 * @param nbrElem      The number of pixels to sprinkle in.
 */
void JNICALL Java_visionanalysis_CryptoboxDetector_salt(JNIEnv *env, jobject instance, jlong matAddrGray, jint nbrElem)
{
    Mat &mGr = *(Mat *) matAddrGray;
    for (int k = 0; k < nbrElem; k++) {
        int i = rand() % mGr.cols;
        int j = rand() % mGr.rows;
        mGr.at<uchar>(j, i) = 255;
    }
}

/**
 * Gets the mat between a threshold of two mats instead of two scalars.
 */
void JNICALL Java_hankextensions_vision_opencv_OpenCVJNIHooks_inRangeBetweenMatsNative(JNIEnv *env, jobject instance, jlong toFilterAddr, jlong lowerAddr, jlong upperAddr, jlong destAddr)
{
    inRange(*(Mat *) toFilterAddr, *(Mat *) lowerAddr, *(Mat *) upperAddr, *(Mat *) destAddr);
}

/**
 * Converts a given RGB mat to CYMK (additive -> subtractive color mixing).
 * @param env
 * @param instance
 * @param original
 */
void JNICALL Java_hankextensions_vision_opencv_OpenCVJNIHooks_cmykConvert(JNIEnv *env, jobject instance, jlong original)
{
    Mat &img = *(Mat *) original;

    // rgb to cmyk
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            Vec3b &color = img.at<Vec3b>(i, j);
            double r = color[0];
            double g = color[1];
            double b = color[2];
            double k = min(min(1 - r, 1 - g), 1 - b);

            color[0] = (1 - r - k) / (1 - k) * 255.;
            color[1] = (1 - g - k) / (1 - k) * 255.;
            color[2] = (1 - b - k) / (1 - k) * 255.;

            img.at<Vec3b>(i, j) = color;
        }
    }
}

/**
 * Once Java's done the simple filtering by ensuring that each positive has some blue and white,
 * this does the more complex filtering (checking each region meets specific criteria, etc.
 *
 * @param cryptoboxMatAddress   The memory address of the cryptobox mat.
 * @param currentPositives      The current array of positive hits.
 */
void JNICALL Java_hankextensions_vision_opencv_OpenCVJNIHooks_deepCryptoboxAnalysis(JNIEnv *env, jobject instance, jlong cryptoboxMatAddress, jlong primaryMaskAddress, jlong whiteMaskAddress, jbooleanArray positives)
{
    // Define mats.
    Mat &cryptoboxMat = *(Mat *) cryptoboxMatAddress;
    Mat &primaryMask = *(Mat *) primaryMaskAddress;
    Mat &whiteMask = *(Mat *) whiteMaskAddress;

    // Define minimum and maximum size for each region.
    double blueRegionMin = (.12) * cryptoboxMat.cols;
    double blueRegionMax = (.7) * cryptoboxMat.cols;
    double whiteRegionMin = (.03) * cryptoboxMat.cols;
    double whiteRegionMax = (.1) * cryptoboxMat.cols;

    // First index of second dimension = distinct valid blue stripes detected, second = distinct valid white stripes detected.
    int representations[cryptoboxMat.rows];

    // Will be calculated during the first run through.
    double averageViewedRegions = 0;
    int totalValidTrials = 0;

    // Obtain references to the boolean array and such (https://www.math.uni-hamburg.de/doc/java/tutorial/native1.1/implementing/array.html)
    jboolean *positivesArrayBody = (*env).GetBooleanArrayElements(positives, 0);

    // Loop through all rows.
    for (int row = 0; row < cryptoboxMat.rows; row++)
    {
        // Ignore non-positives.
        if (!positivesArrayBody[row])
            continue;

        int maxConsecutiveCryptoRegions = 0;
        int consecutiveCryptoRegions = 0;

        int streak = 0, last = 0; // for last, 0 = none, 1 = primary, 2 = white.
        for (int col = 0; col < cryptoboxMat.cols; col++)
        {
            int current = 0;

            if (primaryMask.at<uchar>(row, col) != 0)
                current = 1;
            else if (whiteMask.at<uchar>(row, col) != 0)
                current = 2;

            if (last == current)
            {
                streak++;
            }
            else
            {
                if (streak > 0)
                {
                    if (last == 1 && streak > blueRegionMin && streak < blueRegionMax)
                    {
                        consecutiveCryptoRegions++;
                    }
                    else if (last == 2 && streak > whiteRegionMin && streak < whiteRegionMax)
                    {
                        consecutiveCryptoRegions++;
                    }

                    streak = 0;
                }
            }

            // Start a new consecutive streak if this is a none streak.
            if (current == 0)
            {
                if (consecutiveCryptoRegions > maxConsecutiveCryptoRegions)
                    maxConsecutiveCryptoRegions = consecutiveCryptoRegions;
                consecutiveCryptoRegions = 0;

                streak = 0; // also reset internal count.
            }

            last = current;
        }
        // In case the column was uniform.
        if (streak > 0)
        {
            if (last == 1 && streak > blueRegionMin && streak < blueRegionMax)
            {
                consecutiveCryptoRegions++;
            }
            else if (last == 2 && streak > whiteRegionMin && streak < whiteRegionMax)
            {
                consecutiveCryptoRegions++;
            }
        }
        if (consecutiveCryptoRegions > maxConsecutiveCryptoRegions)
            maxConsecutiveCryptoRegions = consecutiveCryptoRegions;

        if (maxConsecutiveCryptoRegions < 3)
            continue;

        representations[row] = maxConsecutiveCryptoRegions;

        // Apply data to averages
        totalValidTrials++;
        averageViewedRegions = (averageViewedRegions * (totalValidTrials - 1) + maxConsecutiveCryptoRegions) / totalValidTrials;
    }

    // Filter out based on averages.
    for (int row = 0; row < cryptoboxMat.rows; row++)
        if (representations[row] == 0 || abs(representations[row] - averageViewedRegions) > 2)
            positivesArrayBody[row] = false;

    // Has to be done after finishing with the array.
    (*env).ReleaseBooleanArrayElements(positives, positivesArrayBody, 0);
}

}