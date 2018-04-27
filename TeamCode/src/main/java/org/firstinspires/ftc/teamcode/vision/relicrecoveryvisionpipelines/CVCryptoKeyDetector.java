package org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines;

import android.os.Environment;

import dude.makiah.androidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.opencv.core.Core;
import org.opencv.core.DMatch;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.util.List;

import hankutanku.EnhancedOpMode;
import hankutanku.files.FileManager;
import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.opencv.VisionOpMode;

@Autonomous(name="CV CryptoKey Detector", group= OpModeDisplayGroups.INSTANCE.getVISION_TESTING())
public class CVCryptoKeyDetector extends EnhancedOpMode implements VisionOpMode
{
    protected double minDist = 0, maxDist = 4.3; // default settings

    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this);

        ProcessConsole console = log.newProcessConsole("Matching Console");

        while (true)
        {
            if (gamepad1.left_trigger > 0.3)
                minDist -= .001;
            else if (gamepad1.left_bumper)
                minDist += .001;

            if (gamepad1.right_trigger > 0.3)
                maxDist -= .001;
            else if (gamepad1.right_bumper)
                maxDist += .001;

            console.write(
                    "Left is " + resultMatches[0],
                    "Center is " + resultMatches[1],
                    "Right is " + resultMatches[2],
                    "Current is " + getLastDetected().toString(),
                    "Most likely is " + getMostLikelyKey().toString(),
                    "Min = " + minDist,
                    "Max = " + maxDist);

            flow.yield();
        }
    }

    @Override
    public Size idealViewResolution() {
        return new Size(1600, 1300);
    }

    @Override
    public OpenCVCam.CameraPosition viewLocation() {
        return OpenCVCam.CameraPosition.BACK;
    }

    @Override
    public boolean enableCameraFlash() {
        return false;
    }

    // Required for descriptor stuff.
    private FeatureDetector detector;
    private DescriptorExtractor descriptor;
    private DescriptorMatcher matcher;

    private Mat getMatDescriptors(Mat unprocessed)
    {
        MatOfKeyPoint mokp = new MatOfKeyPoint();
        Mat descriptors = new Mat();
        detector.detect(unprocessed, mokp);
        descriptor.compute(unprocessed, mokp, descriptors);
        mokp.release();
        return descriptors;
    }

    private MatOfDMatch getMatchesBetween(Mat descriptors1, Mat descriptors2)
    {
        MatOfDMatch matches = new MatOfDMatch();
        matcher.match(descriptors1, descriptors2, matches);
        return matches;
    }

    // https://ceciliavision.wordpress.com/2014/12/16/feature-matching-in-android/
    private int[] findGoodMatchesFrom(MatOfDMatch[] matchesArray)
    {
        int[] result = new int[3];

        // Determine min distance (will be compared to all)
        double min = Double.MAX_VALUE;
        for (MatOfDMatch matches : matchesArray)
        {
            List<DMatch> matchesList = matches.toList();

            for (DMatch match : matchesList)
            {
                if (match == null)
                    continue;

                if (match.distance < min)
                    min = match.distance;
            }
        }

        // Determine good matches based on min.
        for (int i = 0; i < matchesArray.length; i++)
        {
            List<DMatch> matchesList = matchesArray[i].toList();

            for (int j = 0; j < matchesList.size(); j++)
            {
                if (matchesList.get(j) == null)
                    continue;

                if (minDist * min < matchesList.get(j).distance && matchesList.get(j).distance < maxDist * min)
                    result[i]++;
            }
        }

        // Release all match mats.
        for (MatOfDMatch matches : matchesArray)
        {
            matches.release();
        }

        // hotfix for the right match (for some reason extra sens
        result[2] -= 4;

        return result;
    }

    private int[] determineKeyMatches(MatOfDMatch[] matchesArray)
    {
        int[] results = new int[3];

        // Determine min distance (will be compared to all)
        for (int i = 0; i < results.length; i++)
        {
            List<DMatch> matchesList = matchesArray[i].toList();

            for (DMatch match : matchesList)
            {
                if (match == null)
                    continue;

                if (minDist < match.distance && match.distance < maxDist)
                    results[i]++;
            }
        }

        return results;
    }

    // 0 = left, 1 = center, 2 = right
    private Mat[] loadedImages = new Mat[3];
    private Mat[] keyDescriptors = new Mat[3];
    public int[] resultMatches = new int[3];

    private void copyKeyImagesToPhone() throws IOException
    {
        FileManager.CopyRAWtoSDCard(com.qualcomm.ftcrobotcontroller.R.raw.left, "FIRST/left.png");
        FileManager.CopyRAWtoSDCard(com.qualcomm.ftcrobotcontroller.R.raw.center, "FIRST/center.png");
        FileManager.CopyRAWtoSDCard(com.qualcomm.ftcrobotcontroller.R.raw.right, "FIRST/right.png");
    }

    private Size compareSize;

    @Override
    public void onCameraViewStarted(int width, int height) {
        try
        {
            copyKeyImagesToPhone();
        }
        catch (IOException e) {}

        compareSize = new Size(300, 270);

        detector = FeatureDetector.create(FeatureDetector.AKAZE);
        descriptor = DescriptorExtractor.create(DescriptorExtractor.AKAZE);
        matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_SL2);

        loadedImages[0] = Imgcodecs.imread(Environment.getExternalStorageDirectory() + "/FIRST/left.png", Imgcodecs.CV_LOAD_IMAGE_COLOR);
        loadedImages[1] = Imgcodecs.imread(Environment.getExternalStorageDirectory() + "/FIRST/center.png", Imgcodecs.CV_LOAD_IMAGE_COLOR);
        loadedImages[2] = Imgcodecs.imread(Environment.getExternalStorageDirectory() + "/FIRST/right.png", Imgcodecs.CV_LOAD_IMAGE_COLOR);

        for (int i = 0; i < 3; i++)
            Imgproc.resize(loadedImages[i], loadedImages[i], compareSize);

        for (int i = 0; i < 3; i++)
            keyDescriptors[i] = getMatDescriptors(loadedImages[i]);
    }

    @Override
    public void onCameraViewStopped()
    {
        for (Mat keyDescriptor : keyDescriptors)
            keyDescriptor.release();
    }


    // What this vision pipeline returns to the user.
    public enum DetectedKey { LEFT, CENTER, RIGHT, UNKNOWN }
    private DetectedKey lastDetected = DetectedKey.UNKNOWN;
    public DetectedKey getLastDetected()
    {
        return lastDetected;
    }

    /**
     * Since there's a bit of noise involved in the output for this feature detection, this
     * looks for streaks of data (consecutive detection).
     */
    private static final int REQUIRED_STREAK = 3;
    private DetectedKey mostLikelyKey = DetectedKey.LEFT, currentStreakKey = DetectedKey.LEFT;
    private int currentKeyStreak = 0;
    public DetectedKey getMostLikelyKey()
    {
        return mostLikelyKey;
    }

    @Override
    public Mat onCameraFrame(Mat rgba)
    {
        // Pre-process original frame.
        Mat gray = rgba.clone();
//        Imgproc.cvtColor(gray, gray, Imgproc.COLOR_RGBA2GRAY);
        Imgproc.resize(gray, gray, compareSize);
        Mat currDescriptors = getMatDescriptors(gray);

        // Determine matches.
        MatOfDMatch[] discoveredMatches = new MatOfDMatch[3];
        for (int i = 0; i < keyDescriptors.length; i++)
            discoveredMatches[i] = getMatchesBetween(currDescriptors, keyDescriptors[i]);

        // Filter matches
        resultMatches = findGoodMatchesFrom(discoveredMatches);

        // Predict the current displayed image.
        int greatestIndex = 0;
        for (int i = 1; i < resultMatches.length; i++)
            if (resultMatches[i] > resultMatches[greatestIndex])
                greatestIndex = i;
        switch (greatestIndex)
        {
            case 0:
                lastDetected = DetectedKey.LEFT;
                break;

            case 1:
                lastDetected = DetectedKey.CENTER;
                break;

            case 2:
                lastDetected = DetectedKey.RIGHT;
                break;
        }

        // Release the descriptors
        currDescriptors.release();
        gray.release();

        // Add the type in text.
        Imgproc.putText(rgba, lastDetected.toString(), new Point(rgba.cols() * .5, rgba.rows() * .5), Core.FONT_ITALIC, 1.0, new Scalar(127, 127, 127));

        // Interpret the key and update the most likely key
        if (currentStreakKey == lastDetected)
        {
            currentKeyStreak++;
            if (currentKeyStreak >= REQUIRED_STREAK)
                mostLikelyKey = currentStreakKey;
        }
        else
        {
            currentKeyStreak = 0;
            currentStreakKey = lastDetected;
        }

        return rgba;
    }
}
