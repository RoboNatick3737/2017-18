package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.core.Mat;

import hankextensions.RobotCore;
import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.opencv.OpenCVMatReceiver;

@Autonomous(name="Cryptobox Recognition", group= Constants.EXPERIMENTATION)
public class CryptoboxRecognition extends RobotCore implements OpenCVMatReceiver
{
    private OpenCVCam openCVCam;

    @Override
    protected void INITIALIZE()
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        openCVCam.start();

        openCVCam.setCameraMode(OpenCVCam.CameraMode.REQUEST);
    }

    @Override
    protected void START() throws InterruptedException
    {
        while (true)
        {
            RobotCore.instance.log.lines("Requesting new frame...");
            openCVCam.requestFrame(this);

            while (newMat == null)
                flow.yield();
            Mat currentMat = newMat;
            newMat = null;

            RobotCore.instance.log.lines("Got new frame, looping again...");

            flow.msPause(2000);
        }
    }

    private Mat newMat = null;
    @Override
    public void receiveMat(Mat mat)
    {
        newMat = mat;
    }
}
