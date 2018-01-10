package org.firstinspires.ftc.teamcode.vision.analysis;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.GenericFiltering;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

/**
 * Tracks and guesses the approximate distances from this phone to each individual cryptobox
 * column through a bit of math.
 */
@Autonomous(name="Cryptobox Tracker", group=Constants.EXPERIMENTATION)
public class CryptoboxTracker extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this, true);

        ProcessConsole console = log.newProcessConsole("Cryptobox Tracker");

        while (true)
        {
            console.write("Distances are " + placementDistances[0] + " and " + placementDistances[1] + " and " + placementDistances[2]);
            flow.yield();
        }
    }

    /**
     * Distance to drive forward before hitting cryptobox.
     */
    private double forwardPixelOffset = 0;
    public double getForwardPixelOffset()
    {
        return forwardPixelOffset;
    }

    /**
     * For when we don't have the context required to determine the crypto opening we're most
     * nearly in front of.  1 <= closestGlyphPlacementSpace <= 4.  Can be set by autonomous when we're fairly
     * certain of current location.
     */
    public final int[] placementDistances = new int[3];
    private int lastNumColumnsDetected = 0;

    /**
     * Stores the start of the crypto column and the width of the column.
     */
    class CryptoColumnPixelLocation
    {
        public final int origin; // can't be changed
        public int width; // can be modified

        public CryptoColumnPixelLocation(int origin, int width)
        {
            this.origin = origin;
            this.width = width;
        }

        public double midpoint()
        {
            return this.origin + this.width / 2.0;
        }
    }

    // Mat sizes which constitute analysis vs. the size of the frame we were originally passed.
    private Size originalResolution;
    private Size analysisResolution;

    // Pre-initialized mats.
    private Mat blueMask, whiteMask, cryptoboxMask;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        originalResolution = new Size(width, height);
        analysisResolution = new Size(width, height);

        blueMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        whiteMask = new Mat(analysisResolution, Imgproc.THRESH_BINARY);
        cryptoboxMask = new Mat(analysisResolution, Imgproc.THRESH_BINARY);
    }

    @Override
    public void onCameraViewStopped()
    {
        whiteMask.release();
        blueMask.release();
        cryptoboxMask.release();
    }

    // Disable this if you don't want to display the current mat state to the user (useful for ensuring everything's working properly but slow)
    private final boolean IN_MAT_DEBUG_MODE = true;

    /**
     * Annoying method to find equidistant currentTrackers (likely to therefore represent cryptobox).
     */
    private void getEquidistantColumnsFrom(ArrayList<CryptoColumnPixelLocation> locations)
    {
        // Additional filtering if we detect more than 5 columns.
        final double CLOSE_THRESHOLD = .05 * analysisResolution.width;

        // Find the 4 equidistant columns which represent a cryptobox.
        for (int first = 0; first < locations.size() - 3; first++)
        {
            for (int second = first + 1; second < locations.size() - 2; second++)
            {
                for (int third = second + 1; third < locations.size() - 1; third++)
                {
                    if (!(Math.abs((second - first) - (third - second)) < CLOSE_THRESHOLD))
                        continue;

                    for (int fourth = third + 1; fourth < locations.size(); fourth++)
                    {
                        if (Math.abs((third - second) - (fourth - third)) < CLOSE_THRESHOLD)
                        {
                            for (int i = 0; i < locations.size(); i++)
                            {
                                if (!(i == first || i == second || i == third || i == fourth))
                                {
                                    locations.remove(i);
                                    i--;
                                }
                            }

                            return;
                        }
                    }
                }
            }
        }

        // We haven't found the columns apparently so we'll just remove them from the right end.
        while (locations.size() > 4)
            locations.remove(locations.size() - 1);
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        Core.flip(raw, raw, 1);

        // Set low resolution for analysis (to speed this up)
        Imgproc.resize(raw, raw, analysisResolution);

        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Get the blue mask with adaptive hsv.
        GenericFiltering.blueFilter(raw, blueMask);

        // Get the white mask using just inRange.
        GenericFiltering.whiteFilter(raw, whiteMask);

        // Get cryptobox
        Core.bitwise_or(blueMask, whiteMask, cryptoboxMask);

        // Display the results in the display mat.
        if (IN_MAT_DEBUG_MODE)
        {
            raw.setTo(new Scalar(0, 0, 0));
            raw.setTo(new Scalar(255, 255, 255), whiteMask);
            raw.setTo(new Scalar(0, 0, 255), blueMask);
        }

        // Loop through the columns and eliminate those which aren't cryptobox columns.
        boolean[] cryptoColumns = new boolean[raw.rows()]; // Represents a single-row binary mask.
        for (int i = 0; i < raw.rows(); i++)
        {
            // Count blue and white pixels from binary masks obtained prior.
            int bluePixels = Core.countNonZero(blueMask.row(i));
            int whitePixels = Core.countNonZero(whiteMask.row(i));
            int cryptoboxPixels = Core.countNonZero(cryptoboxMask.row(i));

            // Decide if numbers are correct
            if (!(cryptoboxPixels > .8 * raw.rows() && bluePixels > .5 * raw.rows() && whitePixels > .1 * raw.rows() && bluePixels > 2.5 * whitePixels))
                continue;

            // Decide whether they're mixed appropriately.
            int swaps = 0;
            boolean lastWasWhite = false;
            boolean reachedColumnStart = false;
            for (int j = 0; j < raw.cols(); j++)
            {
                // Decide whether each individual pixel is blue and/or white.
                boolean isBlue = blueMask.get(i, j)[0] != 0, isWhite = whiteMask.get(i, j)[0] != 0;

                // Skip over white pixels.
                if (!reachedColumnStart)
                {
                    reachedColumnStart = isBlue;
                    if (!reachedColumnStart)
                        continue;
                }

                if (!isBlue && !isWhite)
                    continue;

                if (isBlue == isWhite)
                    continue;

                if ((isBlue && !lastWasWhite) || (isWhite && !lastWasWhite))
                {
                    swaps++;

                    if (swaps > 3)
                    {
                        cryptoColumns[i] = true;
                        break;
                    }
                }

                lastWasWhite = isWhite;
            }
        }

        // Find distinct regions in binary array
        ArrayList<CryptoColumnPixelLocation> columns = new ArrayList<>();
        int currLength = 0;
        for (int i = 0; i < cryptoColumns.length; i++)
        {
            if (!cryptoColumns[i]) // when it's false, restart the length count but register this series of trues.
            {
                if (currLength > 0)
                    columns.add(new CryptoColumnPixelLocation(i - currLength, currLength));

                currLength = 0;
            }
            else
                currLength++;
        }
        // Otherwise the last column won't be counted.
        if (currLength > 0)
            columns.add(new CryptoColumnPixelLocation(cryptoColumns.length - currLength - 1, currLength));

        // Merge close currentTrackers (small column blips)
        final int MERGE_THRESHOLD = (int)(analysisResolution.width / 20.0);
        for (int locIndex = 0; locIndex < columns.size() - 1; locIndex++)
        {
            int locOffset = columns.get(locIndex + 1).origin - (columns.get(locIndex).origin + columns.get(locIndex).width);

            if (locOffset < MERGE_THRESHOLD) // if the end of the first is close enough to the start of the second
            {
                columns.get(locIndex).width += locOffset + columns.get(locIndex + 1).width; // increase size of first col
                columns.remove(locIndex + 1); // remove next col
                locIndex--; // have to reduce count because just considered next col already
            }
        }

        // Just exit right here if we haven't detected any columns.
        if (columns.size() == 0)
        {
            // Resize the image to the original size.
            Imgproc.resize(raw, raw, originalResolution);
            return raw;
        }

//        // Display chosen cols in mat if in debug mode in red (might be removed).
//        if (IN_MAT_DEBUG_MODE)
//        {
//            for (CryptoColumnPixelLocation location : columns)
//            {
//                for (int colIndex = 0; colIndex < location.width; colIndex++)
//                {
//                    raw.row(location.origin + colIndex).setTo(new Scalar(255, 0, 0));
//                }
//            }
//        }
//
        // Try to filter out false columns if we detected too many.
        if (columns.size() > 4)
            getEquidistantColumnsFrom(columns);

        // Display chosen cols in mat if in debug mode in green (will be overridden if red).
        if (IN_MAT_DEBUG_MODE)
        {
            for (CryptoColumnPixelLocation location : columns)
            {
                Imgproc.rectangle(raw, new Point(0, location.origin), new Point(raw.cols(), location.origin + location.width), new Scalar(0, 255, 0), 3);
            }
        }

//        if (true)
//        {
//            Imgproc.resize(raw, raw, originalResolution);
//            return raw;
//        }

//        // Use the fact that we've recorded closestGlyphPlacementSpace for previous trials if less than 4 detected.
        int centerScreen = (int)(raw.rows() / 2.0);
        switch (columns.size()) // <= 4
        {
            case 4: // Set all offsets.
                for (int i = 0; i < 3; i++)
                    placementDistances[i] = (int)((columns.get(i).midpoint() + columns.get(i + 1).midpoint()) / 2.0 - centerScreen);

                break;

            case 3:
                if (lastNumColumnsDetected == 4)
                {
                    // Determine which to set to the maximum.
                    int furthestPlacementDist = 0, furthestPlacementIndex = 0;
                    for (int i = 0; i < 3; i++)
                    {
                        if (Math.abs(placementDistances[i]) > Math.abs(furthestPlacementDist))
                        {
                            furthestPlacementIndex = i;
                            furthestPlacementDist = placementDistances[i];
                        }
                    }

                    placementDistances[furthestPlacementIndex] = Integer.MAX_VALUE * (int)(Math.signum(furthestPlacementDist));
                }
                else if (lastNumColumnsDetected == 2)
                {
                    int closestDist = Integer.MAX_VALUE;
                    for (int i = 0; i < 3; i++)
                    {
                        if (Math.abs(placementDistances[i]) < Math.abs(closestDist))
                        {
                            closestDist = placementDistances[i];
                        }
                    }

                    // Decide which one just reappeared.
                    if ((int)(Math.signum(closestDist)) == -1)
                        placementDistances[0] = 40;
                    else if ((int)(Math.signum(closestDist)) == 1)
                        placementDistances[2] = -40;
                }

                if (Math.abs(placementDistances[0]) == Integer.MAX_VALUE) // If we can't see the first column.
                {
                    placementDistances[1] = (int)((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                    placementDistances[2] = (int)((columns.get(1).midpoint() + columns.get(2).midpoint()) / 2.0 - centerScreen);
                }
                else if (Math.abs(placementDistances[2]) == Integer.MAX_VALUE) // If we can't see the last column.
                {
                    placementDistances[0] = (int)((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                    placementDistances[1] = (int)((columns.get(1).midpoint() + columns.get(2).midpoint()) / 2.0 - centerScreen);
                }

                break;

            case 2:
                if (lastNumColumnsDetected == 3)
                {
                    int closestIndex = Integer.MAX_VALUE, closestDist = Integer.MAX_VALUE;
                    for (int i = 0; i < 3; i++)
                    {
                        if (Math.abs(placementDistances[i]) < Math.abs(closestDist))
                        {
                            closestIndex = i;
                            closestDist = placementDistances[i];
                        }
                    }

                    // Set all except closest index to max.
                    for (int i = 0; i < 3; i++)
                    {
                        if (i != closestIndex)
                        {
                            placementDistances[i] = (int)(Math.signum(placementDistances[i])) * Integer.MAX_VALUE;
                        }
                    }
                }

                if (Math.abs(placementDistances[0]) == Integer.MAX_VALUE && Math.abs(placementDistances[1]) == Integer.MAX_VALUE)
                {
                    placementDistances[2] = (int)((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                }
                else if (Math.abs(placementDistances[0]) == Integer.MAX_VALUE && Math.abs(placementDistances[2]) == Integer.MAX_VALUE)
                {
                    placementDistances[1] = (int)((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                }
                else if (Math.abs(placementDistances[1]) == Integer.MAX_VALUE && Math.abs(placementDistances[2]) == Integer.MAX_VALUE)
                {
                    placementDistances[0] = (int)((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                }

                break;
        }

        lastNumColumnsDetected = columns.size();

        // Resize the image to the original size.
        Imgproc.resize(raw, raw, originalResolution);

        return raw;
    }
}
