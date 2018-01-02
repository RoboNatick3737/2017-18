package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;

import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.vuforia.VuforiaCam;
import hankextensions.RobotCore;

@Autonomous(name="Vision Swap", group=Constants.EXPERIMENTATION)
public class VisionSwap extends RobotCore
{
    OpenCVCam openCVCam;
    VuforiaCam vuforiaCam;

    @Override
    protected void onRun() throws InterruptedException
    {
        openCVCam = new OpenCVCam();
        vuforiaCam = new VuforiaCam();

        openCVCam.start();
        log.lines("Started OpenCV");

        waitForStart();

        openCVCam.stop();
        vuforiaCam.start();
        log.lines("Started Vuforia");

        while (true)
            flow.yield();
    }
}
