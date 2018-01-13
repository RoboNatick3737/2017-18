package org.firstinspires.ftc.teamcode.vision.analysis;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Jewel Vision", group= Constants.FINAL_BOT_OPMODES)
public class JewelDetector extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    private OpenCVCam openCVCam;

    private ProcessConsole cameraProcessConsole;

    @Override
    protected void onRun() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        openCVCam.start(this, true);

        cameraProcessConsole = log.newProcessConsole("Camera Process Console");

        waitForStart();

        while (true)
        {
            cameraProcessConsole.write("Current is " + getCurrentOrder().toString());
            flow.yield();
        }
    }

    // From left to right.
    public enum JewelOrder {
        RED_BLUE,
        BLUE_RED,
        UNKNOWN
    }
    // The results.
    private JewelOrder currentOrder = JewelOrder.UNKNOWN;
    public JewelOrder getCurrentOrder() {
        return currentOrder;
    }

    /**
     * References to the jewel submats.
     */
    private class JewelAnalyzer
    {
        public final Rect rect;
        public final Mat blue, red;
        public int bluePixels, redPixels;

        public JewelAnalyzer(Rect rect)
        {
            this.rect = rect;

            this.blue = Mat.zeros(rect.size(), Imgproc.THRESH_BINARY);
            this.red = Mat.zeros(rect.size(), Imgproc.THRESH_BINARY);
        }

        public void analyze(Mat mat)
        {
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2YCrCb);
            LinkedList<Mat> channels = new LinkedList<>();
            Core.split(mat, channels);
            Imgproc.equalizeHist(channels.get(2), channels.get(2));
            Imgproc.threshold(channels.get(2), blue, 160, 255, Imgproc.THRESH_BINARY);
            Imgproc.equalizeHist(channels.get(2), channels.get(2));
            Imgproc.threshold(channels.get(1), red, 160, 255, Imgproc.THRESH_BINARY);
            bluePixels = Core.countNonZero(blue);
            redPixels = Core.countNonZero(red);
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_YCrCb2RGB);

            mat.release();
            for (Mat channel : channels)
                channel.release();
        }

        public boolean isBlue()
        {
            return bluePixels > .4 * rect.area() && !isRed();
        }

        public boolean isRed()
        {
            return redPixels > .4 * rect.area() && !isBlue();
        }

        public void release()
        {
            blue.release();
            red.release();
        }
    }
    private JewelAnalyzer first, second;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        // Define search Rects for jewels.
        first = new JewelAnalyzer(new Rect(new Point(0, height * .26), new Point(width * .07, height * .48)));
        second = new JewelAnalyzer(new Rect(new Point(0, height * .62), new Point(width * .07, height * .84)));
    }

    @Override
    public void onCameraViewStopped()
    {
        first.release();
        second.release();
    }

    @Override
    public Mat onCameraFrame(Mat original)
    {
        // Flip to appear normally on upside-down phone.
        Core.flip(original, original, 1);

        // Remove noise from image.
        Imgproc.blur(original, original, new Size(3, 3));

        // Get red and blue from the rectangle images.
        first.analyze(original.submat(first.rect));
        second.analyze(original.submat(second.rect));

        // Decide which is which.
        if (first.isRed() && second.isBlue())
            currentOrder = JewelOrder.BLUE_RED;
        else if (second.isRed() && first.isBlue())
            currentOrder = JewelOrder.RED_BLUE;
        else
            currentOrder = JewelOrder.UNKNOWN;

        // Color the rectangles appropriately.
        Scalar firstColor = null, secondColor = null;
        switch (currentOrder) {
            case BLUE_RED:
                firstColor = new Scalar(255, 0, 0);
                secondColor = new Scalar(0, 0, 255);
                break;

            case RED_BLUE:
                firstColor = new Scalar(0, 0, 255);
                secondColor = new Scalar(255, 0, 0);
                break;

            case UNKNOWN:
                firstColor = new Scalar(125, 125, 125);
                secondColor = new Scalar(125, 125, 125);
                break;
        }

        // Draw the rectangles.
        Imgproc.rectangle(original, first.rect.tl(), first.rect.br(), firstColor, 3);
        Imgproc.rectangle(original, second.rect.tl(), second.rect.br(), secondColor, 3);

        // Return the resulting mat.
        return original;
    }
}
