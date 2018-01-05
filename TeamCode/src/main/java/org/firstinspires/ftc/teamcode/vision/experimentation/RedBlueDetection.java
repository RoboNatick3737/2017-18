package org.firstinspires.ftc.teamcode.vision.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.structs.LinearFunction;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.AdditionalFilteringUtilities;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.commonareafilter.LinearChannelBound;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.LinearFunctionBounds;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.commonareafilter.ThreeChannelProportionalFilter;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

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

    @Override
    public void onCameraViewStarted(int width, int height) {
        blueMask = Mat.ones(new Size(width, height), Imgproc.THRESH_BINARY);
        redMask = Mat.ones(new Size(width, height), Imgproc.THRESH_BINARY);
    }

    @Override
    public void onCameraViewStopped() {
        redMask.release();
        blueMask.release();
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        AdditionalFilteringUtilities.fixMatLuminance(raw);

        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Analyze frame in HSV
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HSV);

        // An advanced filter I made to choose colors based on the other properties of their hue.
        ThreeChannelProportionalFilter.commonAreaFilter(raw, blueMask,

                // for when we're calculating hue
                new LinearChannelBound(
                        new LinearFunctionBounds(new LinearFunction(0, 0), new LinearFunction(0, 255)),  // describes saturation
                        new LinearFunctionBounds(new LinearFunction(0, 0), new LinearFunction(0, 255))), // describes value

                // for when we're calculating saturation
                new LinearChannelBound(
                        new LinearFunctionBounds(new LinearFunction(0, 0), new LinearFunction(0, 255)),  // describes hue
                        new LinearFunctionBounds(new LinearFunction(0, 0), new LinearFunction(0, 255))), // describes value

                // for when we're calculating value
                new LinearChannelBound(
                        new LinearFunctionBounds(new LinearFunction(0, 75), new LinearFunction(10, 135)),  // describes hue
                        new LinearFunctionBounds(new LinearFunction(0, 59), new LinearFunction(0, 255)))  // describes saturation

                );

        // Set values
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_HSV2RGB);
        raw.setTo(new Scalar(0, 0, 255), blueMask);

        // Resize to the original (crashes otherwise)
        return raw;
    }
}
