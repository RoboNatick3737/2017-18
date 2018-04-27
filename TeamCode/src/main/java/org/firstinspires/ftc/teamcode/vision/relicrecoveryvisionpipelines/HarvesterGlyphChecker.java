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

/**
 * Uses whether or not the yellow stripe on the glyph holder servo is obscured to determine whether
 * we've got two glyphs.
 */
@Autonomous(name="Harvester Glyph Checker", group= OpModeDisplayGroups.INSTANCE.getVISION_TESTING())
public class HarvesterGlyphChecker extends EnhancedOpMode implements VisionOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        OpenCVCam cam = new OpenCVCam();
        cam.start(this);
        ProcessConsole console = log.newProcessConsole("Harvester Glyph Checker");

        while (true)
        {
            console.write(
                    "Harvester glyphs: " + harvestedGlyphs,
                    "Area filled: " + areaFilled);

            flow.yield();
        }
    }

    @Override
    public Size idealViewResolution() {
        return null;
    }

    @Override
    public OpenCVCam.CameraPosition viewLocation() {
        return OpenCVCam.CameraPosition.FRONT;
    }

    @Override
    public boolean enableCameraFlash() {
        return false;
    }

    private Rect glyphLocation;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        glyphLocation = new Rect(new Point(width * .035, height * .33), new Point(width * .22, height * .45));
    }

    @Override
    public void onCameraViewStopped()
    {}

    private double areaFilled = 0;
    private int harvestedGlyphs = 0;
    public int getGlyphsHarvested()
    {
        return harvestedGlyphs;
    }

    @Override
    public Mat onCameraFrame(Mat rgba)
    {
        Imgproc.cvtColor(rgba, rgba, Imgproc.COLOR_RGBA2RGB);

        Core.flip(rgba, rgba, 1);

        Mat glyphMat = rgba.submat(glyphLocation).clone();

        OpenCVJNIHooks.cmykConvert(glyphMat);

        LinkedList<Mat> channels = new LinkedList<>();
        Core.split(glyphMat, channels);
        Mat yellowMat = channels.get(2);

        Mat goodYellow = Mat.zeros(glyphMat.size(), Imgproc.THRESH_BINARY);
        Imgproc.threshold(yellowMat, goodYellow, 200, 255, Imgproc.THRESH_BINARY);

        // Apply mask to visible output.
        rgba.colRange((int)(glyphLocation.tl().x), (int)(glyphLocation.br().x)).rowRange((int)(glyphLocation.tl().y), (int)(glyphLocation.br().y))
                .setTo(new Scalar(255, 125, 0), goodYellow);

        // Draw the location of the glyph submat on screen.
        Imgproc.rectangle(rgba, glyphLocation.tl(), glyphLocation.br(), new Scalar(0, 255, 0), 3);

        // Decide the number of glyphs we have based on the area covered.
        areaFilled = Core.countNonZero(goodYellow) / glyphLocation.area();
        if (areaFilled < .05)
            harvestedGlyphs = 2;
        else if (areaFilled < .4)
            harvestedGlyphs = 1;
        else
            harvestedGlyphs = 0;

        glyphMat.release();
        yellowMat.release();
        goodYellow.release();

        return rgba;
    }
}
