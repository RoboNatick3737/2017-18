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
 * Does the heavy lifting for glyph analysis.
 */
void JNICALL Java_hankutanku_vision_opencv_OpenCVJNIHooks_deepGlyphAnalysis(JNIEnv *env, jobject instance, jlong glyphBinaryMatAddress)
{
    // Get the glyph binary mat.
    Mat& glyphBinaryMat = *(Mat *) glyphBinaryMatAddress;

    // Go through the rows of the mat and decide which one is the primary contour.
}

}