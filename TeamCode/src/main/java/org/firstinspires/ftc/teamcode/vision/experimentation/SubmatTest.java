package org.firstinspires.ftc.teamcode.vision.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Submat Test", group= Constants.VISION_TESTING)
public class SubmatTest extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    Rect region;

    @Override
    public void onCameraViewStarted(int width, int height) {
        region = new Rect(new Point(width * .2, height * .8), new Point(width * .8, height * .2));
    }

    @Override
    public void onCameraViewStopped() {
    }

    @Override
    public Mat onCameraFrame(Mat inputFrame) {

        Mat regionMat = inputFrame.submat(region);
        regionMat.setTo(new Scalar(255, 0, 0));
        Mat ref = inputFrame.colRange((int)(region.tl().x), (int)(region.br().x)).rowRange((int)(region.tl().y), (int)(region.br().y));
        regionMat.copyTo(ref);
        regionMat.release();

        return inputFrame;
    }

    @Override
    protected void onRun() throws InterruptedException {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this, false);

        while(true)
            flow.yield();
    }
}
