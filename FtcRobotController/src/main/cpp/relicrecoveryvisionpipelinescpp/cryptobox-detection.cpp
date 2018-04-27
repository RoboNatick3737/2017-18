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
 * Once Java's done the simple filtering by ensuring that each positive has some blue and white,
 * this does the more complex filtering (checking each region meets specific criteria, etc.
 *
 * @param cryptoboxMatAddress   The memory address of the cryptobox mat.
 * @param currentPositives      The current array of positive hits.
 */
void JNICALL Java_hankutanku_vision_opencv_OpenCVJNIHooks_deepCryptoboxAnalysis(JNIEnv *env, jobject instance, jlong cryptoboxMatAddress, jlong primaryMaskAddress, jlong whiteMaskAddress, jdouble estimatedForwardDistance, jbooleanArray positives)
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