package org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines;

import dude.makiah.androidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

import hankutanku.EnhancedOpMode;
import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.opencv.OpenCVJNIHooks;
import hankutanku.vision.opencv.VisionOpMode;

@Autonomous(name="Jewel Detector", group= OpModeDisplayGroups.INSTANCE.getVISION_TESTING())
public class JewelDetector extends EnhancedOpMode implements VisionOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        OpenCVCam openCVCam = new OpenCVCam();
        openCVCam.start(this);

        ProcessConsole cameraProcessConsole = log.newProcessConsole("Camera Process Console");

        waitForStart();

        while (true)
        {
            cameraProcessConsole.write("Current is " + getCurrentOrder().toString());
            flow.yield();
        }
    }

    @Override
    public Size idealViewResolution()
    {
        return new Size(1600, 1300);
    }

    @Override
    public OpenCVCam.CameraPosition viewLocation()
    {
        return OpenCVCam.CameraPosition.BACK;
    }

    @Override
    public boolean enableCameraFlash()
    {
        return false;
    }

    private enum ColorDetectionMethod { YCrCb, CYMK }
    private final ColorDetectionMethod colorDetectionMethod = ColorDetectionMethod.CYMK;

    private final Scalar
            RED = new Scalar(255, 0, 0),
            BLUE = new Scalar(0, 0, 255),
            GRAY = new Scalar(127, 127, 127);

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
        private final Rect rect;
        public final Mat blue, red;
        private boolean isBlue, isRed;

        public JewelAnalyzer(Rect rect)
        {
            this.rect = rect;

            this.blue = Mat.zeros(rect.size(), Imgproc.THRESH_BINARY);
            this.red = Mat.zeros(rect.size(), Imgproc.THRESH_BINARY);
        }

        /**
         * Pass in the entire image.
         * @param original the entire image originally.
         */
        public void analyze(Mat original)
        {
            Imgproc.cvtColor(original, original, Imgproc.COLOR_RGBA2RGB);

            Mat toAnalyze = original.submat(rect);

            if (colorDetectionMethod == ColorDetectionMethod.YCrCb)
            {
                // Use YCrCb color space
                Imgproc.cvtColor(toAnalyze, toAnalyze, Imgproc.COLOR_RGB2YCrCb);
                LinkedList<Mat> channels = new LinkedList<>();
                Core.split(toAnalyze, channels);

                // Get blue mask
                Imgproc.equalizeHist(channels.get(2), channels.get(2));
                Imgproc.threshold(channels.get(2), blue, 160, 255, Imgproc.THRESH_BINARY);

                // Get red mask
                Imgproc.equalizeHist(channels.get(2), channels.get(2));
                Imgproc.threshold(channels.get(1), red, 160, 255, Imgproc.THRESH_BINARY);
            }
            else
            {
                // Use YCrCb color space
                OpenCVJNIHooks.cmykConvert(toAnalyze);
                LinkedList<Mat> channels = new LinkedList<>();
                Core.split(toAnalyze, channels);

                // Get blue mask
                Imgproc.threshold(channels.get(0), blue, 160, 255, Imgproc.THRESH_BINARY);

                // Get red mask
                Imgproc.threshold(channels.get(1), red, 190, 255, Imgproc.THRESH_BINARY);
            }

            // Decide red and blue.
            int bluePixels = Core.countNonZero(blue), redPixels = Core.countNonZero(red);
            isBlue = bluePixels > .175 * rect.area() && redPixels < .1 * rect.area();
            isRed = redPixels > .175 * rect.area() && bluePixels < .1 * rect.area();

            Imgproc.cvtColor(toAnalyze, toAnalyze, Imgproc.COLOR_YCrCb2RGB);

            // Apply masks to mat
            toAnalyze.setTo(new Scalar(0, 0, 0));
            if (isRed)
            {
                // do blue first so red goes over blue
                toAnalyze.setTo(BLUE, blue);
                toAnalyze.setTo(RED, red);
            }
            else if (isBlue)
            {
                // do red first so blue goes over
                toAnalyze.setTo(RED, red);
                toAnalyze.setTo(BLUE, blue);
            }
            else
            {
                // Default
                toAnalyze.setTo(RED, red);
                toAnalyze.setTo(BLUE, blue);
            }

            // Apply this mat to the original mat.
            Mat copyToPointer = original.colRange((int)(rect.tl().x), (int)(rect.br().x)).rowRange((int)(rect.tl().y), (int)(rect.br().y)); //.setTo(new Scalar(0, 0, 0));
            toAnalyze.copyTo(copyToPointer);

            // Release the mat which we're analyzing.
            toAnalyze.release();
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
        first = new JewelAnalyzer(new Rect(new Point(width * .11, height * .32), new Point(width * .21, height * .45)));
        second = new JewelAnalyzer(new Rect(new Point(width * .11, height * .55), new Point(width * .21, height * .76)));
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
        Imgproc.cvtColor(original, original, Imgproc.COLOR_RGBA2RGB);

        // Flip to appear normally on upside-down phone.
//        Core.flip(original, original, 1);

        // Remove noise from image.
        Imgproc.blur(original, original, new Size(3, 3));

        // Get red and blue from the rectangle images.
        first.analyze(original);
        second.analyze(original);

        // Decide which is which.
        if (first.isRed && second.isBlue)
            currentOrder = JewelOrder.BLUE_RED;
        else if (second.isRed && first.isBlue)
            currentOrder = JewelOrder.RED_BLUE;
        else
            currentOrder = JewelOrder.UNKNOWN;

        // Color the rectangles appropriately.
        Scalar firstColor = null, secondColor = null;
        switch (currentOrder) {
            case BLUE_RED:
                firstColor = RED;
                secondColor = BLUE;
                break;

            case RED_BLUE:
                firstColor = BLUE;
                secondColor = RED;
                break;

            case UNKNOWN:
                firstColor = GRAY;
                secondColor = GRAY;
                break;
        }

        // Draw the rectangles.
        Imgproc.rectangle(original, first.rect.tl(), first.rect.br(), firstColor, 3);
        Imgproc.rectangle(original, second.rect.tl(), second.rect.br(), secondColor, 3);

        // Return the resulting mat.
        return original;
    }
}
