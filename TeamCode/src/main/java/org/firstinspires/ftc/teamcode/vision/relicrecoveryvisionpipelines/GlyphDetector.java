package org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import hankutanku.EnhancedOpMode;
import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.opencv.VisionOpMode;

@Autonomous(name="Glyph Detector", group= OpModeDisplayGroups.INSTANCE.getVISION_TESTING())
public class GlyphDetector extends EnhancedOpMode implements VisionOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this);

        while (true)
            flow.yield();
    }

    @Override
    public Size idealViewResolution()
    {
        return new Size(1300, 1000); // high res
    }

    @Override
    public OpenCVCam.CameraPosition viewLocation()
    {
        return OpenCVCam.CameraPosition.BACK;
    }

    @Override
    public boolean enableCameraFlash() {
        return false;
    }

    @Override
    public void onCameraViewStarted(int width, int height)
    {
    }

    @Override
    public void onCameraViewStopped()
    {
    }

    @Override
    public Mat onCameraFrame(Mat rgba)
    {
        // Remove alpha channel.
        Imgproc.cvtColor(rgba, rgba, Imgproc.COLOR_RGBA2RGB);

        // This is where the crypto key shows up when we start the robot on the balancing board.
        Rect chosenSubmat = new Rect(new Point(rgba.cols() * .4, rgba.rows() * .3), new Point(rgba.cols() * .55, rgba.rows() * .4));

        Mat submat = rgba.submat(chosenSubmat);

        Imgproc.cvtColor(submat, submat, Imgproc.COLOR_RGB2GRAY);

        // Find edges around the glyphs.
        Imgproc.adaptiveThreshold(submat, submat, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, 11, 7);

        // Use OpenCV to figure out whether we should be rotating around the glyph to get a better attack angle.

        // The pointer mat.
        Mat pointerMat = rgba.colRange((int)(chosenSubmat.tl().x), (int)(chosenSubmat.br().x)).rowRange((int)(chosenSubmat.tl().y), (int)(chosenSubmat.br().y));

        // Apply adaptive threshold mask to pointer mat
        pointerMat.setTo(new Scalar(0, 0, 0));
        pointerMat.setTo(new Scalar(255, 255, 255), submat);

        // Release the processed frame.
        submat.release();

        return rgba;
    }
}
