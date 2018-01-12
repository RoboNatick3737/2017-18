package org.firstinspires.ftc.teamcode.vision.analysis;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.LinkedList;

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
            console.write("Distances are " + placementDistances[0] + ", " + placementDistances[1] + " and " + placementDistances[2]);
            flow.yield();
        }
    }

    // The alliance for which we'll be doing vision.
    private CompetitionProgram.Alliance alliance = CompetitionProgram.Alliance.BLUE;
    public void setAlliance(CompetitionProgram.Alliance alliance)
    {
        this.alliance = alliance;
    }

    /**
     * For when we don't have the context required to determine the crypto opening we're most
     * nearly in front of.  1 <= closestGlyphPlacementSpace <= 4.  Can be set by autonomous when we're fairly
     * certain of current location.
     */
    public final int[] placementDistances = new int[3];
    private int lastNumColumnsDetected = 0;
    private int framesSinceColumnsDetected = 0;

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
    private Mat primaryMask, whiteMask, cryptoboxMask;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        originalResolution = new Size(width, height);
        analysisResolution = new Size(width, height);

        primaryMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        whiteMask = new Mat(analysisResolution, Imgproc.THRESH_BINARY);
        cryptoboxMask = new Mat(analysisResolution, Imgproc.THRESH_BINARY);
    }

    @Override
    public void onCameraViewStopped()
    {
        whiteMask.release();
        primaryMask.release();
        cryptoboxMask.release();
    }

    // Disable this if you don't want to display the current mat state to the user (useful for ensuring everything's working properly but slow)
    private final boolean IN_MAT_DEBUG_MODE = true;

    /**
     * Converts image to the YCrCb color space and filters by channel to determine red/blue
     * and white filters.
     */
    private void updateColorMasks(Mat main)
    {
        Imgproc.cvtColor(main, main, Imgproc.COLOR_RGB2YCrCb);
        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(main, channels);
        if (alliance == CompetitionProgram.Alliance.BLUE)
        {
            Mat blue = channels.get(2);
            Imgproc.equalizeHist(blue, blue);
            Imgproc.threshold(blue, primaryMask, 210, 255, Imgproc.THRESH_BINARY);
        }
        else if (alliance == CompetitionProgram.Alliance.RED)
        {
            Mat red = channels.get(1);
            Imgproc.equalizeHist(red, red);
            Imgproc.threshold(red, primaryMask, 180, 255, Imgproc.THRESH_BINARY);
        }
        Mat white = channels.get(0);
        Imgproc.equalizeHist(white, white);
        Imgproc.threshold(white, whiteMask, 160, 255, Imgproc.THRESH_BINARY);
        Imgproc.cvtColor(main, main, Imgproc.COLOR_YCrCb2RGB);

        Core.bitwise_or(primaryMask, whiteMask, cryptoboxMask);
    }

    enum CryptoColor {PRIMARY, WHITE, NONE}
    class ColorRegion
    {
        private CryptoColor color;
        private int length;

        public ColorRegion(CryptoColor color, int length)
        {
            this.color = color;
            this.length = length;
        }
    }

    /**
     * Takes a simplified column of crypto colors and decides whether it belongs to a cryptobox.
     */
    private boolean isCryptoColumn(int rowOfMat)
    {
        /////  SIMPLE FILTER (between a cryptobox column and background objects) //////

        // Get height of frame (all measurements relative to this).
        int height = cryptoboxMask.cols();

        // Get the num of mask pixels in each region.
        int cryptoPixels = Core.countNonZero(cryptoboxMask.row(rowOfMat)),
                primaryPixels = Core.countNonZero(primaryMask.row(rowOfMat)),
                whitePixels = Core.countNonZero(whiteMask.row(rowOfMat));

        // Allow additional leniency in cryptobox height depending on how long it's been since we saw the cryptobox.
        double filterFactor = (4 - lastNumColumnsDetected) - framesSinceColumnsDetected * .5;
        if (filterFactor < 0)
            filterFactor = 0;

        // Don't analyze if this obviously isn't a column.
        if (!(cryptoPixels > (.4 + filterFactor * .03) * height &&
                primaryPixels > (.3 + filterFactor * .02)  * height &&
                whitePixels > (.02 + filterFactor * .01) * height))
            return false;


        ///// COMPLEX FILTER (between a cryptobox column and a similarly colored pant leg) ///////

        // Generate a simplified list of colors to define the column.
        CryptoColor[] simplifiedColumn = new CryptoColor[height];
        for (int j = 0; j < height; j++)
            simplifiedColumn[j] = (whiteMask.get(rowOfMat, j)[0] != 0) ? CryptoColor.WHITE : ((primaryMask.get(rowOfMat, j)[0] != 0) ? CryptoColor.PRIMARY : CryptoColor.NONE);

        // Determine the distinct regions in the column and their lengths.
        ArrayList<ColorRegion> regions = new ArrayList<>();
        int streak = 0;
        CryptoColor last = simplifiedColumn[0];
        for (int i = 0; i < height; i++)
        {
            if (simplifiedColumn[i] != last)
            {
                regions.add(new ColorRegion(simplifiedColumn[i - 1], streak));
                streak = 0;
            }

            streak++;
            last = simplifiedColumn[i];
        }
        if (streak > 0)
            regions.add(new ColorRegion(last, streak));

        // Eliminate super small regions.
        for (int i = 0; i < regions.size(); i++)
        {
            if (regions.get(i).length < (0.01 + filterFactor * .005) * height) // Has to be at least 3% length.
            {
                regions.remove(i);
                i--;
            }
        }

        // Merge common regions.
        for (int i = 0; i < regions.size() - 1; i++)
        {
            if (regions.get(i).color == regions.get(i + 1).color)
            {
                regions.get(i).length += regions.get(i + 1).length;
                regions.remove(i + 1);
                i--;
            }
        }

        // Remove NONE bounds from ends.
        if (regions.get(regions.size() - 1).color == CryptoColor.NONE)
            regions.remove(regions.size() - 1);
        if (regions.get(0).color == CryptoColor.NONE)
            regions.remove(0);

        // Decide whether these regions indicate a cryptobox.
        int maxBlueStripeSequence = 0, maxWhiteStripeSequence = 0;
        int whiteStripesVisible = 0, blueStripesVisible = 0;
        CryptoColor lastRegionColor = CryptoColor.NONE;
        for (ColorRegion region : regions)
        {
            if (region.color == CryptoColor.NONE && region.length > .1 * simplifiedColumn.length)
            {
                if (blueStripesVisible > maxBlueStripeSequence)
                    maxBlueStripeSequence = blueStripesVisible;
                if (whiteStripesVisible > maxWhiteStripeSequence)
                    maxWhiteStripeSequence = whiteStripesVisible;

                whiteStripesVisible = 0;
                blueStripesVisible = 0;
            }
            else if (region.color == CryptoColor.PRIMARY && lastRegionColor != CryptoColor.PRIMARY
                    && region.length > (.1 + filterFactor * .025) * simplifiedColumn.length)
            {
                blueStripesVisible++;
            }
            else if (region.color == CryptoColor.WHITE && lastRegionColor != CryptoColor.WHITE
                    && region.length > (.02 + filterFactor * .005) * simplifiedColumn.length)
            {
                whiteStripesVisible++;
            }

            lastRegionColor = region.color;
        }
        if (blueStripesVisible > 0 || whiteStripesVisible > 0)
        {
            if (blueStripesVisible > maxBlueStripeSequence)
                maxBlueStripeSequence = blueStripesVisible;
            if (whiteStripesVisible > maxWhiteStripeSequence)
                maxWhiteStripeSequence = whiteStripesVisible;
        }

        return maxBlueStripeSequence >= maxWhiteStripeSequence && maxBlueStripeSequence >= 2 && maxWhiteStripeSequence >= 1;
    }

    /**
     * Simplifies the one layer boolean array into locations.
     */
    private ArrayList<CryptoColumnPixelLocation> discoverLocationsFrom(boolean[] cryptoColumns)
    {
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

        return columns;
    }

    /**
     * Complicated method to find equidistant currentTrackers (likely to therefore represent
     * cryptobox).
     */
    private void filterEquidistantColumnsFrom(ArrayList<CryptoColumnPixelLocation> locations)
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

    /**
     * Used for finding the closest magnitude of placement distance (from [-30, 40, MAX] returns -30)
     */
    private int getSmallestMagnitudeFrom(int[] array)
    {
        // Find closest distance for tracking.
        int closestIndex = 0;
        for (int i = 1; i < array.length; i++)
            if (Math.abs(placementDistances[i]) < Math.abs(placementDistances[closestIndex]))
                closestIndex = i;

        return closestIndex;
    }

    private boolean isValidLocation(int location)
    {
        return Math.abs(location) != Integer.MAX_VALUE;
    }

    private void setPlacementLocationIfPossible(int locationIndex, int value)
    {
        if (locationIndex >= 0 && locationIndex < placementDistances.length)
            placementDistances[locationIndex] = value;
    }

    /**
     * Tries to determine placement locations based on the detected columns.
     */
    private void decidePlacementLocationsBasedOn(ArrayList<CryptoColumnPixelLocation> columns)
    {
        // Use the fact that we've recorded closestGlyphPlacementSpace for previous trials if less than 4 detected.
        int centerScreen = (int)(cryptoboxMask.rows() / 2.0);
        switch (columns.size()) // <= 4
        {
            /**
             * There's no real difficulty here, just set each placement distance based on the
             * detected columns.
             */
            case 4:
                for (int i = 0; i < 3; i++)
                    placementDistances[i] = (int)((columns.get(i).midpoint() + columns.get(i + 1).midpoint()) / 2.0 - centerScreen);

                break;

            /**
             * This is a bit harder.
             *
             * If we're transitioning from 4 detected columns to 3, we
             * decide whether the first or last placement location was further away last time,
             * and set that to the max value.  It's less hard because there are only 2 locations
             * it might not be seeing.
             *
             * If we're transitioning from 2 columns, we have to first determine which placement
             * location was the visible one, and then determine which column just reappeared based
             * on that column's offset.
             */
            case 3:
                // We'd seen every column prior.
                if (lastNumColumnsDetected == 4)
                {
                    if (Math.abs(placementDistances[0]) > Math.abs(placementDistances[2]))
                        placementDistances[0] = -Integer.MAX_VALUE;
                    else
                        placementDistances[2] = Integer.MAX_VALUE;
                }
                // We'd only seen one placement box prior.
                else if (lastNumColumnsDetected == 2)
                {
                    // We were just looking at the first placement location, so set middle to +40
                    if (isValidLocation(placementDistances[0]))
                    {
                        placementDistances[1] = 40;
                    }
                    // We were just looking at the third placement location, so set middle to -40
                    else if (isValidLocation(placementDistances[2]))
                    {
                        placementDistances[1] = -40;
                    }
                    // We were just looking at the middle one, so we'll enable first or last depending on the placement dist to te center.
                    else if (isValidLocation(placementDistances[1]))
                    {
                        if (placementDistances[1] < 0)
                            placementDistances[2] = 40;
                        else if (placementDistances[1] > 0)
                            placementDistances[2] = -40;
                    }
                }

                // If we can't see the first column
                if (!isValidLocation(placementDistances[0]))
                {
                    placementDistances[1] = (int)((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                    placementDistances[2] = (int)((columns.get(1).midpoint() + columns.get(2).midpoint()) / 2.0 - centerScreen);
                }
                // If we can't see the last column.
                else if (!isValidLocation(placementDistances[2]))
                {
                    placementDistances[0] = (int)((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                    placementDistances[1] = (int)((columns.get(1).midpoint() + columns.get(2).midpoint()) / 2.0 - centerScreen);
                }

                break;

            /**
             * For 3 -> 2, we have to figure out the closest placement box, and the resulting locations of the others.
             */
            case 2:
                if (lastNumColumnsDetected == 3 || lastNumColumnsDetected == 1)
                {
                    int closestIndex = getSmallestMagnitudeFrom(placementDistances);

                    // Set all except closest index to max.
                    for (int i = 0; i < 3; i++)
                        if (i != closestIndex)
                            placementDistances[i] = (int)(Math.signum(placementDistances[i])) * Integer.MAX_VALUE;
                }

                // Set the one that isn't the max value to the observed value.
                for (int i = 0; i < 3; i++)
                {
                    if (isValidLocation(placementDistances[i]))
                    {
                        placementDistances[i] = (int) ((columns.get(0).midpoint() + columns.get(1).midpoint()) / 2.0 - centerScreen);
                        break;
                    }
                }

                break;

            /**
             * For this one, we just track how the position of one is changing and set the positions
             * arbitrarily high or low but with the correct signs.
             */
            case 1:
                // Decide which two placement locations we'll be deciding between.
                if (lastNumColumnsDetected == 2)
                {
                    int previouslyVisiblePlacementLocation = getSmallestMagnitudeFrom(placementDistances);

                    if (placementDistances[previouslyVisiblePlacementLocation] < 0)
                    {
                        if (previouslyVisiblePlacementLocation < 2)
                            placementDistances[previouslyVisiblePlacementLocation + 1] = 40;
                    }
                    else if (placementDistances[previouslyVisiblePlacementLocation] > 0)
                    {
                        if (previouslyVisiblePlacementLocation > 0)
                            placementDistances[previouslyVisiblePlacementLocation - 1] = -40;
                    }
                    else // This is [hopefully] impossible, it would detect three columns.
                    {}
                }

                // The two indices which are smallest.
                int validIndex1 = -1; // Left one
                int validIndex2 = -1; // Right one.
                if (isValidLocation(placementDistances[0]))
                {
                    if (isValidLocation(placementDistances[1]))
                    {
                        validIndex1 = 0;
                        validIndex2 = 1;
                    }
                    else
                    {
                        // edge placement
                        validIndex1 = -1;
                        validIndex2 = 0;
                    }
                }
                else if (isValidLocation(placementDistances[2]))
                {
                    if (isValidLocation(placementDistances[1]))
                    {
                        validIndex1 = 1;
                        validIndex2 = 2;
                    }
                    else
                    {
                        validIndex1 = 2; // this must be rightmost since other is edge.
                        validIndex2 = 3;
                    }
                }

                int observedLocation = (int)(columns.get(0).midpoint()) - centerScreen;

                if (observedLocation == 0)
                {
                    setPlacementLocationIfPossible(validIndex1, -30);
                    setPlacementLocationIfPossible(validIndex2, 30);
                }
                else if (observedLocation > 0) // on left part of screen so right one is closer.
                {
                    setPlacementLocationIfPossible(validIndex1, -30);
                    setPlacementLocationIfPossible(validIndex2, 40);
                }
                else
                {
                    setPlacementLocationIfPossible(validIndex2, -40);
                    setPlacementLocationIfPossible(validIndex2, 30);
                }

                break;
        }
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        // Flip so that it appears normally on the RC.
        Core.flip(raw, raw, 1);

        // Set low resolution for analysis (to speed this up)
        Imgproc.resize(raw, raw, analysisResolution);

        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Blue/red and white filtering.
        updateColorMasks(raw);

        // Display the results in the display mat.
        if (IN_MAT_DEBUG_MODE)
        {
            raw.setTo(new Scalar(0, 0, 0));
            raw.setTo(new Scalar(255, 255, 255), whiteMask);
            raw.setTo(new Scalar(0, 0, 255), primaryMask);
        }

        // Loop through the columns and eliminate those which aren't cryptobox columns.
        boolean[] cryptoColumns = new boolean[raw.rows()]; // Represents a single-row binary mask.
        for (int i = 0; i < raw.rows(); i++)
            cryptoColumns[i] = isCryptoColumn(i);

        // Discover the distinct regions of the array from the resulting boolean array.
        ArrayList<CryptoColumnPixelLocation> columns = discoverLocationsFrom(cryptoColumns);

        // Just exit right here if we haven't detected any columns.
        if (columns.size() == 0)
        {
            framesSinceColumnsDetected++;

            // Resize the image to the original size.
            Imgproc.resize(raw, raw, originalResolution);
            return raw;
        }

        // Try to filter out false columns if we detected too many.
        if (columns.size() > 4)
            filterEquidistantColumnsFrom(columns);

        // Display chosen cols in mat if in debug mode in green (will be overridden if red).
        if (IN_MAT_DEBUG_MODE)
            for (CryptoColumnPixelLocation location : columns)
                Imgproc.rectangle(raw, new Point(0, location.origin), new Point(raw.cols(), location.origin + location.width), new Scalar(0, 255, 0), 3);

        // Decides what the placement locations should be based on the detected columns.
        decidePlacementLocationsBasedOn(columns);

        // Used for future runs.
        lastNumColumnsDetected = columns.size();
        framesSinceColumnsDetected = 0;

        // Resize the image to the original size.
        Imgproc.resize(raw, raw, originalResolution);

        return raw;
    }
}
