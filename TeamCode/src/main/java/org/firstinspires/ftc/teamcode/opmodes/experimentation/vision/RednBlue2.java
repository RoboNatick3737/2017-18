package org.firstinspires.ftc.teamcode.opmodes.experimentation.vision;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.opencv.analysis.MaskGenerator;

@Autonomous(name="Red and Blue 2", group= Constants.EXPERIMENTATION)
public class RednBlue2 extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
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
    private MaskGenerator maskGenerator;

    @Override
    public void onCameraViewStarted(int width, int height) {
        redMask = new Mat();
        blueMask = new Mat();
        maskGenerator = new MaskGenerator(new Size(width, height));
    }

    @Override
    public void onCameraViewStopped() {
        redMask.release();
        blueMask.release();
        maskGenerator.releaseMats();
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        // Make colors appear sharper.
        maskGenerator.fixMatLuminance(raw);

        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Analyze frame in HSV
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HSV);

        // Get the blue mask with adaptive hsv.
        maskGenerator.adaptiveHSV(raw, 55, 75, -.1, 135, -.1, 59, 255, blueMask);

        // Get the red mask with adaptive hsv.
        maskGenerator.adaptiveHSV(raw, 55, 0, 0, 50, .1, 50, 255, redMask);

        // Set values
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_HSV2RGB);
        raw.setTo(new Scalar(0, 0, 0)); // clear existent values
        raw.setTo(new Scalar(0, 0, 255), blueMask);
        raw.setTo(new Scalar(255, 0, 0), redMask);

        // Resize to the original (crashes otherwise)
        return raw;
    }
}
