package visionanalysis;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

public class CryptoboxDetector
{
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat matGray = inputFrame.gray();
        salt(matGray.getNativeObjAddr(), 2000);
        return matGray;
    }

    public native void salt(long matAddrGray, int nbrElem);
}
