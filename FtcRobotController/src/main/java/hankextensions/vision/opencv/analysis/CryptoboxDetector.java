package hankextensions.vision.opencv.analysis;

import org.opencv.core.Mat;

public class CryptoboxDetector
{
    // Example method.
    public native void salt(long matAddrGray, int nbrElem);

    public void filterForCryptobox(Mat mat)
    {
        filterForCrypto(mat.getNativeObjAddr());
    }

    public native void filterForCrypto(long matAddress);
}
