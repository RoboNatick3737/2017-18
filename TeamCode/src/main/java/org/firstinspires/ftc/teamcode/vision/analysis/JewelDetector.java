package org.firstinspires.ftc.teamcode.vision.analysis;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

/**
 * Originally developed by Alex from DogeCV, I'm modifying a bit for my own purposes: https://github.com/GTHSRobotics/DogeCV
 */
public class JewelDetector extends EnhancedOpMode implements CameraBridgeViewBase.CvCameraViewListener
{
    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this);

        ProcessConsole console = LoggingBase.instance.newProcessConsole("Jewel Detector");

        while (true)
        {
            console.write("Order is " + getCurrentOrder().toString());
            flow.yield();
        }
    }

    // From left to right.
    public enum JewelOrder {
        RED_BLUE,
        BLUE_RED,
        UNKNOWN
    }

    public enum JewelDetectionMode {
        PERFECT_AREA, MAX_AREA
    }

    public enum JewelDetectionSpeed {
        VERY_FAST, FAST, BALANCED, SLOW, VERY_SLOW
    }

    // Constant variables over the algorithm's progression.
    public static final JewelDetectionMode  detectionMode    = JewelDetectionMode.MAX_AREA;
    public static final double              downScaleFactor  = 0.4;
    public static final boolean             rotateMat        = false;
    public static final JewelDetectionSpeed speed            = JewelDetectionSpeed.BALANCED;
    public static final double              perfectArea      = 6500;
    public static final double              areaWeight       = 0.05; // Since we're dealing with 100's of pixels
    public static final double              minArea          = 700;
    public static final double              ratioWeight      = 15; // Since most of the time the area diffrence is a decimal place
    public static final double              maxDiffrence     = 10; // Since most of the time the area diffrence is a decimal place
    public static final boolean             debugContours    = false;

    // The results of the mass.
    private JewelOrder currentOrder = JewelOrder.UNKNOWN;
    private JewelOrder lastOrder    = JewelOrder.UNKNOWN;

    // Required variables for analysis.
    private Mat workingMat, blurredMat, maskRed, maskBlue, hierarchy, redConvert, blueConvert;
    private Size newSize, initSize;


    @Override
    public void onCameraViewStarted(int width, int height)
    {
        workingMat = new Mat();
        blurredMat = new Mat();
        maskRed = new Mat();
        maskBlue = new Mat();
        hierarchy = new Mat();

        newSize = new Size();
        initSize = new Size(width, height);
    }

    @Override
    public void onCameraViewStopped()
    {
        workingMat.release();
        blurredMat.release();
        maskRed.release();
        maskBlue.release();
        hierarchy.release();
        redConvert.release();
        blueConvert.release();
    }

    @Override
    public Mat onCameraFrame(Mat rgba)
    {
        newSize  = new Size(initSize.width * downScaleFactor, initSize.height * downScaleFactor);
        rgba.copyTo(workingMat);

        Imgproc.resize(workingMat, workingMat,newSize);

        if(rotateMat){
            Mat tempBefore = workingMat.t();

            Core.flip(tempBefore, workingMat, 1); //mRgba.t() is the transpose

            tempBefore.release();
        }

        redConvert = workingMat.clone();
        blueConvert = workingMat.clone();

        getRedMask(redConvert);
        getBlueMask(blueConvert);


        List<MatOfPoint> contoursRed = new ArrayList<>();

        Imgproc.findContours(maskRed, contoursRed, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat,contoursRed,-1,new Scalar(230,70,70),2);
        Rect chosenRedRect = null;
        double chosenRedScore = Integer.MAX_VALUE;

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for(MatOfPoint c : contoursRed) {
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            // You can find this by printing the area of each found rect, then looking and finding what u deem to be perfect.
            // Run this with the bot, on a balance board, with jewels in their desired location. Since jewels should mostly be
            // in the same position, this hack could work nicely.


            double area = Imgproc.contourArea(c);
            double areaDiffrence = 0;

            switch(detectionMode){
                case MAX_AREA:
                    areaDiffrence = -area * areaWeight;
                    break;
                case PERFECT_AREA:
                    areaDiffrence = Math.abs(perfectArea - area);
                    break;
            }

            // Just declaring vars to make my life eassy
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));

            double cubeRatio = Math.max(Math.abs(h/w), Math.abs(w/h)); // Get the ratio. We use max in case h and w get swapped??? it happens when u account for rotation
            double ratioDifference = Math.abs(cubeRatio - 1);
            double finalDifference = (ratioDifference * ratioWeight) + (areaDiffrence * areaWeight);


            // Optional to ALWAYS return a result.

            // Update the chosen rect if the diffrence is lower then the curreny chosen
            // Also can add a condition for min diffrence to filter out VERY wrong answers
            // Think of diffrence as score. 0 = perfect
            if(finalDifference < chosenRedScore && finalDifference < maxDiffrence && area > minArea)
            {
                chosenRedScore = finalDifference;
                chosenRedRect = rect;
            }

            if(debugContours && area > 100)
            {
                Imgproc.circle(workingMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(workingMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }

        }

        List<MatOfPoint> contoursBlue = new ArrayList<>();

        Imgproc.findContours(maskBlue, contoursBlue, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat,contoursBlue,-1,new Scalar(70,130,230),2);
        Rect chosenBlueRect = null;
        double chosenBlueScore = Integer.MAX_VALUE;

        for(MatOfPoint c : contoursBlue) {
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            // You can find this by printing the area of each found rect, then looking and finding what u deem to be perfect.
            // Run this with the bot, on a balance board, with jewels in their desired location. Since jewels should mostly be
            // in the same position, this hack could work nicely.


            double area = Imgproc.contourArea(c);
            double areaDiffrence = 0;

            switch(detectionMode){
                case MAX_AREA:
                    areaDiffrence = -area * areaWeight;
                    break;
                case PERFECT_AREA:
                    areaDiffrence = Math.abs(perfectArea - area);
                    break;
            }


            // Just declaring vars to make my life eassy
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));

            double cubeRatio = Math.max(Math.abs(h/w), Math.abs(w/h)); // Get the ratio. We use max in case h and w get swapped??? it happens when u account for rotation
            double ratioDifference = Math.abs(cubeRatio - 1);

            double finalDifference = (ratioDifference * ratioWeight) + (areaDiffrence * areaWeight);

            // Update the chosen rect if the diffrence is lower then the curreny chosen
            // Also can add a condition for min diffrence to filter out VERY wrong answers
            // Think of diffrence as score. 0 = perfect
            if(finalDifference < chosenBlueScore && finalDifference < maxDiffrence && area > minArea)
            {
                chosenBlueScore = finalDifference;
                chosenBlueRect = rect;
            }

            if(debugContours && area > 100)
            {
                Imgproc.circle(workingMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(workingMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }

        }

        if(chosenRedRect != null)
        {
            Imgproc.rectangle(workingMat,
                    new Point(chosenRedRect.x, chosenRedRect.y),
                    new Point(chosenRedRect.x + chosenRedRect.width, chosenRedRect.y + chosenRedRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(workingMat,
                    "Red: " + chosenRedScore,
                    new Point(chosenRedRect.x - 5, chosenRedRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(255, 0, 0),
                    2);
        }

        if(chosenBlueRect != null)
        {
            Imgproc.rectangle(workingMat,
                    new Point(chosenBlueRect.x, chosenBlueRect.y),
                    new Point(chosenBlueRect.x + chosenBlueRect.width, chosenBlueRect.y + chosenBlueRect.height),
                    new Scalar(0, 0, 255), 2);

            Imgproc.putText(workingMat,
                    "Blue: " + chosenRedScore,
                    new Point(chosenBlueRect.x - 5, chosenBlueRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 0, 255),
                    2);
        }

        if(chosenBlueRect != null && chosenRedRect != null)
        {
            if(chosenBlueRect.x < chosenRedRect.x)
            {
                currentOrder = JewelOrder.BLUE_RED;
                lastOrder = currentOrder;
            }
            else
            {
                currentOrder = JewelOrder.RED_BLUE;
                lastOrder = currentOrder;
            }
        }
        else
        {
            currentOrder = JewelOrder.UNKNOWN;
        }

        Imgproc.resize(workingMat, workingMat, initSize);

        Imgproc.putText(workingMat,"Result: " + lastOrder.toString(),new Point(10,newSize.height - 30),0,1, new Scalar(255,255,0),1);
        Imgproc.putText(workingMat,"Current Track: " + currentOrder.toString(),new Point(10,newSize.height - 10),0,0.5, new Scalar(255,255,255),1);

        Imgproc.putText(workingMat,"DogeCV JewelV1: " + newSize.toString() + " - " + speed.toString() + " - " + detectionMode.toString() ,new Point(5,15),0,0.6,new Scalar(0,255,255),2);

        return workingMat;
    }

    private void getRedMask(Mat input)
    {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab);
        Imgproc.GaussianBlur(input,input,new Size(3,3),0);
        List<Mat> channels = new ArrayList<Mat>();
        Core.split(input, channels);
        Imgproc.threshold(channels.get(1), maskRed, 164.0, 255, Imgproc.THRESH_BINARY);
    }

    private void getBlueMask(Mat input)
    {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
        Imgproc.GaussianBlur(input,input,new Size(3,3),0);
        List<Mat> channels = new ArrayList<Mat>();
        Core.split(input, channels);
        Imgproc.threshold(channels.get(1), maskBlue, 145.0, 255, Imgproc.THRESH_BINARY);
    }

    public JewelOrder getCurrentOrder() {
        return currentOrder;
    }

    public JewelOrder getLastOrder() {
        return lastOrder;
    }
}