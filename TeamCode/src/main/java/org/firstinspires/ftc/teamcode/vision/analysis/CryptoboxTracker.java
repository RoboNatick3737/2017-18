package org.firstinspires.ftc.teamcode.vision.analysis;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.structs.LinearFunction;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.AdditionalFilteringUtilities;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.LinearFunctionBounds;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.MaskGenerator;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.commonareafilter.LinearChannelBound;
import org.firstinspires.ftc.teamcode.vision.filteringutilities.commonareafilter.ThreeChannelProportionalFilter;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
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
@Autonomous(name="Cryptobox Tracker", group= Constants.EXPERIMENTATION)
public class CryptoboxTracker extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this);

        ProcessConsole console = LoggingBase.instance.newProcessConsole("Cryptobox Tracker");

        while (true)
        {
            console.write("Front is " + getCurrentForwardOffset(), "Side is " + getCurrentHorizontalOffset());
            flow.yield();
        }
    }

    private CryptoboxPositionTracker tracker;
    private MaskGenerator maskGenerator;

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
    /**
     * Tracks a column position with an id
     */
    public class CryptoboxPositionTracker
    {
        private CryptoColumnPixelLocation[] trackedLocations;

        public CryptoboxPositionTracker(CryptoColumnPixelLocation[] locations)
        {
            trackedLocations = locations;
        }

        public void applyFilteredData(CryptoColumnPixelLocation[] observedLocations)
        {
            for (int observedLocationsIndex = 0; observedLocationsIndex < observedLocations.length; observedLocationsIndex++)
            {
                for (int trackedLocationsIndex = 0; trackedLocationsIndex < trackedLocations.length; trackedLocationsIndex++)
                {
                    CryptoColumnPixelLocation trackedPosition = trackedLocations[trackedLocationsIndex];
                    CryptoColumnPixelLocation observedPosition = observedLocations[observedLocationsIndex];

                    if (Math.abs(trackedPosition.midpoint() - observedPosition.midpoint()) < analysisResolution.width * .03)
                    {
                        trackedLocations[trackedLocationsIndex] = observedLocations[observedLocationsIndex];
                        double offset = observedPosition.midpoint() - trackedPosition.midpoint();

                        // Apply offset to all.
                        for (int i = 0; i < trackedLocations.length; i++)
                        {
                            if (i != trackedLocationsIndex) // apply to all except current.
                                trackedLocations[i] = new CryptoColumnPixelLocation((int)(trackedLocations[i].origin + offset), trackedLocations[i].width);
                        }

                        break;
                    }
                }
            }
        }

        public double getHorizontalOffset()
        {
            if (tracker == null)
                return 0;

            return (trackedLocations[1].midpoint() + trackedLocations[2].midpoint()) / 2.0 - analysisResolution.width / 2.0;
        }

        public double getForwardOffset()
        {
            if (tracker == null)
                return 100;

            double avgWidth = 0;
            for (CryptoColumnPixelLocation tracker : trackedLocations)
                avgWidth += tracker.width;
            avgWidth /= trackedLocations.length;

            return .05 * avgWidth;
        }
    }

    /**
     * The end doubles that this complex system is trying to calculate.
     */
    public double getCurrentHorizontalOffset()
    {
        return tracker.getHorizontalOffset();
    }

    public double getCurrentForwardOffset()
    {
        return tracker.getForwardOffset();
    }

    /**
     * Forward dist = dist we can drive forward before hitting the crypto, horizontal dist =
     * dist to the right we can drive before we can reach the center.
     */
    public void provideApproximatePhysicalOffset(double forwardDist, double horizontalOffset)
    {
        CryptoColumnPixelLocation[] locations = new CryptoColumnPixelLocation[4];
        tracker = new CryptoboxPositionTracker(locations);
    }


    // Mat sizes which constitute analysis vs. the size of the frame we were originally passed.
    private Size originalResolution;
    private Size analysisResolution;

    // Pre-initialized mats.
    private Mat blueMask, whiteMask;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        originalResolution = new Size(width, height);
        analysisResolution = new Size(width, height);
        maskGenerator = new MaskGenerator(analysisResolution);

        blueMask = Mat.zeros(analysisResolution, Imgproc.THRESH_BINARY); // 1-channel = grayscale image
        whiteMask = new Mat(analysisResolution, Imgproc.THRESH_BINARY);
    }

    @Override
    public void onCameraViewStopped()
    {
        whiteMask.release();
        blueMask.release();
        maskGenerator.releaseMats();
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
        // Set low resolution for analysis (to speed this up)
        Imgproc.resize(raw, raw, analysisResolution);

        // Make colors appear sharper.
        AdditionalFilteringUtilities.fixMatLuminance(raw);

        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));

        // Analyze frame in HSV
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HSV);

        // Generate blue and white
        ThreeChannelProportionalFilter.commonAreaFilter(raw, blueMask,

                // for when we're calculating hue
                null,

                // for when we're calculating saturation
                null,

                // for when we're calculating value
                new LinearChannelBound(
                        new LinearFunctionBounds(new LinearFunction(-.02, 7.227), new LinearFunction(.77, 134.7)),  // describes hue
                        new LinearFunctionBounds(new LinearFunction(-.0449, 20.62), new LinearFunction(.585, 255)))  // describes saturation

        );
        ThreeChannelProportionalFilter.commonAreaFilter(raw, whiteMask,

                // for when we're calculating hue
                null,

                // for when we're calculating saturation
                null,

                // for when we're calculating value
                new LinearChannelBound(
                        new LinearFunctionBounds(new LinearFunction(.166, 82.26), new LinearFunction(.696, 247.37)),  // describes hue
                        new LinearFunctionBounds(new LinearFunction(.754, 8.67), new LinearFunction(.573, 255)))  // describes saturation

        );

//        // Get the blue mask with adaptive hsv.
//        maskGenerator.adaptiveHSV(raw, 55, -.1, 135, -.1, 59, 59, 255, blueMask);
//
//        // Get the white mask using just inRange.
//        Core.inRange(raw, new Scalar(0, 0, 60), new Scalar(255, 65, 255), whiteMask);

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
        boolean[] cryptoColumns = new boolean[raw.cols()]; // Represents a single-row binary mask.
        for (int colIndex = 0; colIndex < raw.cols(); colIndex++)
        {
            // Count blue and white pixels from binary masks obtained prior.
            int bluePixels = Core.countNonZero(blueMask.col(colIndex));
            int whitePixels = Core.countNonZero(whiteMask.col(colIndex));

            // Neutralize column if criteria isn't fit.
            boolean sufficientBluePixels = bluePixels > .4 * raw.rows();
            boolean sufficientWhitePixels = whitePixels > .1 * raw.rows() && whitePixels < .5 * bluePixels;

            if (sufficientBluePixels && sufficientWhitePixels)
                cryptoColumns[colIndex] = true;
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

        // Display chosen cols in mat if in debug mode in red (might be removed).
        if (IN_MAT_DEBUG_MODE)
        {
            for (CryptoColumnPixelLocation location : columns)
            {
                for (int colIndex = 0; colIndex < location.width; colIndex++)
                {
                    raw.col(location.origin + colIndex).setTo(new Scalar(255, 0, 0));
                }
            }
        }

        // Try to filter out false columns if we detected too many.
        if (columns.size() > 4)
            getEquidistantColumnsFrom(columns);

        // Display chosen cols in mat if in debug mode in green (will be overridden if red).
        if (IN_MAT_DEBUG_MODE)
        {
            for (CryptoColumnPixelLocation location : columns)
            {
                for (int colIndex = 0; colIndex < location.width; colIndex++)
                {
                    raw.col(location.origin + colIndex).setTo(new Scalar(0, 255, 0));
                }
            }
        }

        // Switch to array
        CryptoColumnPixelLocation[] locations = new CryptoColumnPixelLocation[4];
        for (int i = 0; i < 4; i++)
            locations[i] = columns.get(i);

        switch (locations.length) // less than 4
        {
            case 4:
                tracker = new CryptoboxPositionTracker(locations);
                break;

            default:
                if (tracker != null)
                    tracker.applyFilteredData(locations);
        }

        // Resize the image to the original size.
        Imgproc.resize(raw, raw, originalResolution);

        return raw;
    }
}
