#include <jni.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

extern "C"
{

// More of an example of how Mat modification works in opencv c++
void JNICALL Java_visionanalysis_CryptoboxDetector_salt(JNIEnv *env, jobject instance,
                                                                           jlong matAddrGray,
                                                                           jint nbrElem) {
    Mat &mGr = *(Mat *) matAddrGray;
    for (int k = 0; k < nbrElem; k++) {
        int i = rand() % mGr.cols;
        int j = rand() % mGr.rows;
        mGr.at<uchar>(j, i) = 255;
    }
}

// Gets the mat between a threshold of two mats instead of two scalars.
void JNICALL Java_hankextensions_vision_opencv_OpenCVJNIHooks_inRangeBetweenMatsNative(JNIEnv *env, jobject instance, jlong toFilterAddr, jlong lowerAddr, jlong upperAddr, jlong destAddr)
{
    inRange(*(Mat *) toFilterAddr, *(Mat *) lowerAddr, *(Mat *) upperAddr, *(Mat *) destAddr);
}

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

}