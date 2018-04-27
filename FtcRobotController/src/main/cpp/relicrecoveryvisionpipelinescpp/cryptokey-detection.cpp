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
JNIEXPORT jint JNICALL Java_hankutanku_vision_opencv_OpenCVJNIHooks_decideOnCryptokeyIndex(JNIEnv *env, jobject instance, jlong currentDescriptorsAddress, jlong leftDescriptorsAddress, jlong centerDescriptorsAddress, jlong rightDescriptorsAddress)
{
    // Define descriptors.
    Mat& currentDescriptors = *(Mat *) currentDescriptorsAddress;
    Mat& leftDescriptors = *(Mat *) leftDescriptorsAddress;
    Mat& centerDescriptors = *(Mat *) centerDescriptorsAddress;
    Mat& rightDescriptors = *(Mat *) rightDescriptorsAddress;

    // Define descriptors.
    Mat keyDescriptors[3];
    keyDescriptors[0] = leftDescriptors;
    keyDescriptors[1] = centerDescriptors;
    keyDescriptors[2] = rightDescriptors;

    for (int keyDescriptorIndex = 0; keyDescriptorIndex < 3; keyDescriptorIndex++) {
        // Match the descriptors.
        BFMatcher matcher(NORM_HAMMING);
        vector<vector<DMatch> > nn_matches;
        matcher.knnMatch(currentDescriptors, keyDescriptors[keyDescriptorIndex], nn_matches, 2);

        vector<KeyPoint> matched1, matched2, inliers1, inliers2;
        int match1 = 0, match2 = 0;
        vector<DMatch> good_matches;
        for (size_t i = 0; i < nn_matches.size(); i++) {
            DMatch first = nn_matches[i][0];
            float dist1 = nn_matches[i][0].distance;
            float dist2 = nn_matches[i][1].distance;

            if (dist1 < 0.8f * dist2) {
//                matched1.push_back(kpts1[first.queryIdx]);
//                matched2.push_back(kpts2[first.trainIdx]);
            }
        }

        for (unsigned i = 0; i < matched1.size(); i++) {
            Mat col = Mat::ones(3, 1, CV_64F);
            col.at<double>(0) = matched1[i].pt.x;
            col.at<double>(1) = matched1[i].pt.y;

//            col = homography * col;
            col /= col.at<double>(2);
            double dist = sqrt(pow(col.at<double>(0) - matched2[i].pt.x, 2) +
                               pow(col.at<double>(1) - matched2[i].pt.y, 2));

            if (dist < 2.5f) {
                int new_i = static_cast<int>(inliers1.size());
                inliers1.push_back(matched1[i]);
                inliers2.push_back(matched2[i]);
                good_matches.push_back(DMatch(new_i, new_i, 0));
            }
        }

//        Mat res;
//        drawMatches(img1, inliers1, img2, inliers2, good_matches, res);
//        imwrite("res.png", res);

//        double inlier_ratio = inliers1.size() * 1.0 / matched1.size();
//        cout << "A-KAZE Matching Results" << endl;
//        cout << "*******************************" << endl;
//        cout << "# Keypoints 1:                        \t" << kpts1.size() << endl;
//        cout << "# Keypoints 2:                        \t" << kpts2.size() << endl;
//        cout << "# Matches:                            \t" << matched1.size() << endl;
//        cout << "# Inliers:                            \t" << inliers1.size() << endl;
//        cout << "# Inliers Ratio:                      \t" << inlier_ratio << endl;
//        cout << endl;
    }


    return 0;
}

}