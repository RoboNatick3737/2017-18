package org.firstinspires.ftc.teamcode.opmodes.experimentation.unittesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import hankutanku.EnhancedOpMode;
import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.opencv.VisionOpMode;

@Autonomous(name="CV Test", group= OpModeDisplayGroups.VISION_TESTING)
public class SimpleCV extends EnhancedOpMode implements VisionOpMode
{
    @Override
    public Size idealViewResolution() {
        return null;
    }

    @Override
    public OpenCVCam.CameraPosition viewLocation() {
        return OpenCVCam.CameraPosition.BACK;
    }

    @Override
    public boolean enableCameraFlash() {
        return false;
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(Mat inputFrame) {
        return inputFrame;
    }

    @Override
    protected void onRun() throws InterruptedException {
        OpenCVCam cam = new OpenCVCam();

        cam.start(this);

        while (true)
            flow.yield();
    }
}
