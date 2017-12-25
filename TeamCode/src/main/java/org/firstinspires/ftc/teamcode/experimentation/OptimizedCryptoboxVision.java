package org.firstinspires.ftc.teamcode.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.LinkedList;

import hankextensions.RobotCore;
import hankextensions.vision.opencv.OpenCVCam;
import visionanalysis.OpenCVJNIHooks;

@Autonomous(name="Optimized Cryptobox Vision", group= Constants.EXPERIMENTATION)
public class OptimizedCryptoboxVision extends RobotCore implements CameraBridgeViewBase.CvCameraViewListener
{
    private OpenCVCam openCVCam;

    private ProcessConsole cameraProcessConsole;

    @Override
    protected void INITIALIZE() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        openCVCam.start(this);
    }

    @Override
    protected void START() throws InterruptedException
    {
        while (true)
            flow.yield();
    }


    /////// Analysis ///////

    private Size originalResolution;
    private Size analysisResolution;

    // Not constant mats
    private Mat luminanceFix, resultingLower, resultingUpper, blueMask, whiteMask;

    private boolean[] cryptoColumns;

    // A LinkedList into which channels can be split.
    private LinkedList<Mat> channels;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        originalResolution = new Size(width, height);
        analysisResolution = new Size(width, height);

        // init non-constant mats.
        luminanceFix = new Mat(analysisResolution, CvType.CV_8UC1);
        resultingLower = new Mat(analysisResolution, CvType.CV_8UC1);
        resultingUpper = new Mat(analysisResolution, CvType.CV_8UC1);
        blueMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        whiteMask = new Mat(analysisResolution, Imgproc.THRESH_BINARY);

        // Init channels list
        channels = new LinkedList<>();
    }

    @Override
    public void onCameraViewStopped()
    {
        resultingLower.release();
        resultingUpper.release();
        luminanceFix.release();
        whiteMask.release();
        blueMask.release();

        channels = null;
    }

    private void fixMatLuminance(Mat raw)
    {
        // Fix the lighting contrast that results from using different fields.
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2YCrCb);
        Core.split(raw, channels);
        Imgproc.equalizeHist(channels.get(0), channels.get(0));
        Core.merge(channels, raw);
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_YCrCb2RGB);
    }

    /**
     * Stores the start of the crypto column in a single row binary array.
     */
    class CryptoColumnLocation
    {
        public final int origin; // can't be changed
        public int width; // can be modified

        public CryptoColumnLocation(int origin, int width)
        {
            this.origin = origin;
            this.width = width;
        }
    }

    // Disable this if you don't want to display the current mat state to the user (useful for ensuring everything's working properly but slow)
    private final boolean IN_MAT_DEBUG_MODE = true;

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        // Set low resolution for analysis (to speed this up)
        Imgproc.resize(raw, raw, analysisResolution);

        // Make colors appear sharper.
        fixMatLuminance(raw);

        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Analyze frame in HSV
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HSV);

        // Determine the upper and lower mat bounds for blue.
        Core.split(raw, channels);
        Mat hueChannel = channels.get(0);
        Mat saturationChannel = channels.get(1);
        Mat valueChannel = channels.get(2);

        // Adaptive value threshold for blue
        Core.subtract(valueChannel, new Scalar(55), valueChannel); // luminance - 55
        Core.multiply(valueChannel, new Scalar(-.1), luminanceFix); // -.1 * (luminance - 55)
        Core.add(luminanceFix, new Scalar(75), resultingLower); // blue hue > 75 + -.1 * (luminance - 55)
        Core.add(luminanceFix, new Scalar(135), resultingUpper); // blue hue < 135 + -.1 * (luminance - 55)
        OpenCVJNIHooks.inRangeBetweenMats(hueChannel, resultingLower, resultingUpper, blueMask); // resulting blue mask

        // Saturation has to be less than 59 for blue.
        Core.inRange(saturationChannel, new Scalar(59), new Scalar(255), saturationChannel); // for blue sat > 59

        // Get final mask (fits both value and saturation criteria).
        Core.bitwise_and(saturationChannel, blueMask, blueMask);

        // Get the white mask.
        Core.inRange(raw, new Scalar(0, 0, 49), new Scalar(255, 59, 255), whiteMask);

        // Now convert back to normal mode while displaying mats.
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_HSV2RGB);

        // Display the results in the display mat.
        if (IN_MAT_DEBUG_MODE)
        {
            raw.setTo(new Scalar(0, 0, 0));
            raw.setTo(new Scalar(255, 255, 255), whiteMask);
            raw.setTo(new Scalar(0, 0, 255), blueMask);
        }

        // Loop through the columns and eliminate those which aren't cryptobox columns.
        cryptoColumns = new boolean[raw.cols()]; // Represents a single-row binary mask.
        for (int colIndex = 0; colIndex < raw.cols(); colIndex++)
        {
            // Count blue and white pixels from binary masks obtained prior.
            int bluePixels = Core.countNonZero(blueMask.col(colIndex));
            int whitePixels = Core.countNonZero(whiteMask.col(colIndex));

            // Neutralize column if criteria isn't fit.
            if (bluePixels > .5 * raw.rows() && whitePixels > .1 * raw.rows())
                cryptoColumns[colIndex] = true;
        }

        // Find distinct regions in binary array
        ArrayList<CryptoColumnLocation> locations = new ArrayList<>();
        int currLength = 0;
        for (int i = 0; i < cryptoColumns.length; i++)
        {
            if (!cryptoColumns[i]) // when it's false, restart the length count but register this series of trues.
            {
                if (currLength > 0)
                    locations.add(new CryptoColumnLocation(i - currLength, currLength));

                currLength = 0;
            }
            else
                currLength++;
        }

        // Merge close locations (small column blips)
        final int MERGE_THRESHOLD = (int)(analysisResolution.width / 20.0);
        for (int locIndex = 0; locIndex < locations.size() - 1; locIndex++)
        {
            int locOffset = locations.get(locIndex + 1).origin - (locations.get(locIndex).origin + locations.get(locIndex).width);

            if (locOffset < MERGE_THRESHOLD) // if the end of the first is close enough to the start of the second
            {
                locations.get(locIndex).width += locOffset + locations.get(locIndex + 1).width; // increase size of first col
                locations.remove(locIndex + 1); // remove next col
                locIndex--; // have to reduce count because just considered next col already
            }
        }

        // Display chosen cols in mat if in debug mode in green.
        if (IN_MAT_DEBUG_MODE)
        {
            for (CryptoColumnLocation location : locations)
            {
                for (int colIndex = 0; colIndex < location.width; colIndex++)
                {
                    raw.col(location.origin + colIndex).setTo(new Scalar(0, 255, 0));
                }
            }
        }

        // Remove small columns if we're detecting more than 4.
        while (locations.size() > 4)
        {
            int smallestIndex = 0;
            for (int i = 1; i < locations.size(); i++)
            {
                if (locations.get(smallestIndex).width > locations.get(i).width)
                    smallestIndex = i;
            }

            // Display the removed column in red (may or may not exist).
            if (IN_MAT_DEBUG_MODE)
            {
                CryptoColumnLocation toBeRemovedLoc = locations.get(smallestIndex);

                for (int colIndex = 0; colIndex < toBeRemovedLoc.width; colIndex++)
                {
                    raw.col(toBeRemovedLoc.origin + colIndex).setTo(new Scalar(255, 0, 0));
                }
            }

            locations.remove(smallestIndex);
        }

        // Resize the image to the original size.
        Imgproc.resize(raw, raw, originalResolution);
        return raw;
    }
}
