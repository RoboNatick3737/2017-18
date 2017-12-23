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

void JNICALL Java_visionanalysis_CryptoboxDetector_filterForCrypto(JNIEnv *env, jobject instance, jlong matAddr)
{
    Mat &hsv = *(Mat *) matAddr;

    // TODO all C++ vision pipeline code
}

}