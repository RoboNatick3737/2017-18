package org.firstinspires.ftc.teamcode.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

import hankextensions.RobotCore;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Mat Filter — Manual", group= Constants.EXPERIMENTATION)
public class MatFilterManual extends RobotCore implements CameraBridgeViewBase.CvCameraViewListener
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

    // Analysis variables (declare once to improve run speed).
    enum CryptoColor {PRIMARY, WHITE, NONE}

    private Size originalResolution;
    private Size analysisResolution;
    private CryptoColor[][] cryptoColors;
    private Mat cryptoColorMat;

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        originalResolution = new Size(width, height);
        analysisResolution = originalResolution;

        cryptoColors = new CryptoColor[(int)(analysisResolution.width)][(int)(analysisResolution.height)];
        cryptoColorMat = new Mat((int)(analysisResolution.height), (int)(analysisResolution.width), CvType.CV_8UC3);
    }

    @Override
    public void onCameraViewStopped()
    {
        cryptoColorMat.release();
    }

    public Mat getMatFromCryptoColors(CryptoColor[][] cryptoColors)
    {
        // Square.
        int cols = cryptoColors.length;
        int rows = cryptoColors[0].length;

        for (int row = 0; row < rows; row++)
        {
            for (int col = 0; col < cols; col++)
            {
                switch (cryptoColors[col][row])
                {
                    case PRIMARY:
                        cryptoColorMat.put(row, col, 100, 255, 255);
                        break;

                    case WHITE:
                        cryptoColorMat.put(row, col, 0, 0, 255);
                        break;

                    case NONE:
                        cryptoColorMat.put(row, col, 0, 0, 0);
                        break;
                }
            }
        }

        return cryptoColorMat;
    }

    private CryptoColor getProminentColorFrom(CryptoColor[] colors)
    {
        int primaries = 0, whites = 0, nones = 0;

        for (CryptoColor color : colors)
        {
            switch (color)
            {
                case PRIMARY:
                    primaries++;
                    break;

                case WHITE:
                    whites++;
                    break;

                case NONE:
                    nones++;
                    break;
            }
        }

        // Return prominent color.
        if (primaries > whites && primaries > nones)
            return CryptoColor.PRIMARY;
        else if (whites > primaries && whites > nones)
            return CryptoColor.WHITE;
        else
            return CryptoColor.NONE;
    }

    private CryptoColor[] blurColumnArray(CryptoColor[] pixelColumn, int radius)
    {
        // Sort of "blur" the barcode type array with a radius of 5.
        for (int i = 0; i < pixelColumn.length - radius; i += radius)
        {
            CryptoColor[] radiusArray = new CryptoColor[radius];
            for (int j = 0; j < radius; j++)
                radiusArray[j] = pixelColumn[i + j];

            CryptoColor prominentColor = getProminentColorFrom(radiusArray);

            for (int j = 0; j < radius; j++)
                pixelColumn[i + j] = prominentColor;
        }

        return pixelColumn;
    }

    @Override
    public Mat onCameraFrame(Mat raw)
    {
        // Set low resolution for analysis (to speed this up)
        Imgproc.resize(raw, raw, analysisResolution);


        // Fix the lighting contrast that results from using different fields.
        LinkedList<Mat> channels = new LinkedList<>();
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2YCrCb);
        Core.split(raw, channels);
        Imgproc.equalizeHist(channels.get(0), channels.get(0));
        Core.merge(channels, raw);
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_YCrCb2RGB);


        // Remove noise from image.
        Imgproc.blur(raw, raw, new Size(3, 3));


        // Analyze frame in HSV
        Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2HSV);

        // Process the original image into CryptoColors.
        for (int colIndex = 0; colIndex < raw.cols(); colIndex++)
        {
            CryptoColor[] pixelColumn = new CryptoColor[raw.height()];

            // Define the pixel column
            for (int rowIndex = 0; rowIndex < raw.rows(); rowIndex++)
            {
                double[] pixel = raw.get(rowIndex, colIndex);

                double changeFactor = .1 * (pixel[2] - 55);

                if ((75 - changeFactor < pixel[0] && pixel[0] < 135 - changeFactor) && 59 < pixel[1])
                    pixelColumn[rowIndex] = CryptoColor.PRIMARY;

                else if (pixel[1] < 59 && 49 < pixel[2])
                    pixelColumn[rowIndex] = CryptoColor.WHITE;

                else
                    pixelColumn[rowIndex] = CryptoColor.NONE;
            }

            // Short circuit to the next loop if we detect very few blue pixels.
//            if (bluePixels < camResolution.height / 5 && whitePixels < camResolution.height / 10)
//                continue;

            cryptoColors[colIndex] = pixelColumn;
        }

        cryptoColorMat = getMatFromCryptoColors(cryptoColors);
        Imgproc.resize(cryptoColorMat, cryptoColorMat, originalResolution);
        return cryptoColorMat;
    }
}
