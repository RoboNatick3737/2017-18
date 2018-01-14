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
void JNICALL Java_hankextensions_vision_opencv_OpenCVJNIHooks_deepCryptoboxAnalysis(JNIEnv *env, jobject instance, jlong cryptoboxMatAddress, jlong primaryMaskAddress, jlong whiteMaskAddress, jdouble estimatedForwardDistance, jbooleanArray positives)
{
    // Define mats.
    Mat &cryptoboxMat = *(Mat *) cryptoboxMatAddress;
    Mat &primaryMask = *(Mat *) primaryMaskAddress;
    Mat &whiteMask = *(Mat *) whiteMaskAddress;

    // Obtain references to the boolean array and such (https://www.math.uni-hamburg.de/doc/java/tutorial/native1.1/implementing/array.html)
    jboolean *positivesArrayBody = (*env).GetBooleanArrayElements(positives, 0);


    //////// STAGE 1: Filter out columns with a lot of noise (solidity indicates greater likelihood of being a crypto column) ////////
    for (int row = 0; row < cryptoboxMat.rows; row++)
    {
        // Don't count rows which shouldn't be filtered.
        if (!positivesArrayBody[row])
            continue;

        // Discover the number of color changes which took place.
        int colorChanges = 0;
        int lastValid = 0; // 0 = none, 1 = primary, 2 = white
        for (int col = 0; col < cryptoboxMat.cols; col++)
        {
            int current = primaryMask.at<uchar>(row, col) != 0 ? 1 : (whiteMask.at<uchar>(row, col) != 0 ? 2 : 0);

            if (current == 0)
                continue;

            if (current > 0) // valid
            {
                if (lastValid != current)
                    colorChanges++;

                lastValid = current;
            }
        }

        // Remove those with too much noise.
        int excessiveNoiseThreshold = (int)(20 - (44 - estimatedForwardDistance) * .1);
        if (colorChanges > excessiveNoiseThreshold) // Where 10 is some arbitrary constant which indicates a lot of noise.
            positivesArrayBody[row] = false;
    }

    //////// STAGE 2: Ensure that the now-filtered columns are relatively consistent across rows (remove those which aren't) ////////
    /*int conformityScores[cryptoboxMat.rows];
    for (int col = 0; col < cryptoboxMat.cols; col++)
    {
        // Determine the most popular pixel.
        int mostPopularPixel = 0; // 0 = none, 1 = primary, 2 = white
        int results[3]; // first index = none etc.
        for (int row = 0; row < cryptoboxMat.rows; row++)
        {
            if (!positivesArrayBody[row])
                continue;

            int current = primaryMask.at<uchar>(row, col) != 0 ? 1 : (whiteMask.at<uchar>(row, col) != 0 ? 2 : 0);
            results[current]++;
        }
        // Use the array to figure it out.
        for (int i = 1; i < 3; i++)
            if (results[i] > results[mostPopularPixel])
                mostPopularPixel = i;

        // Update similarity scores based on the most popular pixel.
        for (int row = 0; row < cryptoboxMat.rows; row++)
        {
            if (!positivesArrayBody[row])
                continue;

            int current = primaryMask.at<uchar>(row, col) != 0 ? 1 : (whiteMask.at<uchar>(row, col) != 0 ? 2 : 0);
            if (current == mostPopularPixel)
                conformityScores[row]++;
        }
    }

    // There will be some discrepancies due to white stripes seeming diagonal on outermost columns, so be lenient.
    for (int row = 0; row < cryptoboxMat.rows; row++)
        if (positivesArrayBody[row] && conformityScores[row] < .8 * cryptoboxMat.cols)
            positivesArrayBody[row] = false;*/

    // Has to be done after finishing with the array.
    (*env).ReleaseBooleanArrayElements(positives, positivesArrayBody, 0);
}

}