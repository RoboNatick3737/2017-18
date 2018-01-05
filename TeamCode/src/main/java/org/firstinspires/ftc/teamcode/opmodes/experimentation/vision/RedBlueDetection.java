package org.firstinspires.ftc.teamcode.opmodes.experimentation.vision;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.structs.LinearFunction;
import org.firstinspires.ftc.teamcode.vision.MaskGenerator2;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

import org.firstinspires.ftc.teamcode.vision.HSVMaskBound;

@Autonomous(name="Red and Blue Detection", group= Constants.EXPERIMENTATION)
public class RedBlueDetection extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    private OpenCVCam openCVCam;

    private ProcessConsole cameraProcessConsole;

    @Override
    protected void onRun() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        openCVCam.start(this);

        cameraProcessConsole = log.newProcessConsole("Camera Process Console");

        waitForStart();

        while (true)
        {
            flow.yield();
        }
    }

    private Mat redMask, blueMask;
    private MaskGenerator2 maskGenerator;

    @Override
    public void onCameraViewStarted(int width, int height) {
        blueMask = Mat.ones(new Size(width, height), Imgproc.THRESH_BINARY);
        redMask = Mat.ones(new Size(width, height), Imgproc.THRESH_BINARY);
        maskGenerator = new MaskGenerator2();
    }

    @Override
    public void onCameraViewStopped() {
        redMask.release();
        blueMask.release();
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Analyze frame in HSV
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HSV);

        // Get the blue mask with adaptive hsv.
//        maskGenerator.adaptiveHSV(raw, 55, 75, -.1, 135, -.1, 59, 255, blueMask);
        maskGenerator.commonAreaFilter(
                raw, blueMask,
                new HSVMaskBound(new LinearFunction(0, 0), new LinearFunction(0, 255)), // for hue
                new HSVMaskBound(new LinearFunction(0, 59), new LinearFunction(0, 255)), // for saturation
                new HSVMaskBound(new LinearFunction(-.1, 80), new LinearFunction(-.1, 140))); // for value: as odd as this seems, this actually mandates the hue.

        // Get the red mask with adaptive hsv.
//        maskGenerator.adaptiveHSV(raw, 55, 0, 0, 20, .1, 59, 255, redMask);
//
//        // Set values
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_HSV2RGB);
//        raw.setTo(new Scalar(0, 0, 0)); // clear existent values
        raw.setTo(new Scalar(0, 0, 255), blueMask);
//        raw.setTo(new Scalar(255, 0, 0), redMask);

        // Resize to the original (crashes otherwise)
        return raw;
    }
}
