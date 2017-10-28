package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;

import hankextensions.vision.OpenCVCam;
import hankextensions.vision.VuforiaCam;
import hankextensions.Core;
import hankextensions.threading.Flow;

@Autonomous(name="Vision Swap", group=Constants.EXPERIMENTATION)
public class VisionSwap extends Core
{
    OpenCVCam openCVCam;
    VuforiaCam vuforiaCam;

    @Override
    protected void INITIALIZE() throws InterruptedException
    {
        openCVCam = new OpenCVCam();
        vuforiaCam = new VuforiaCam();

        openCVCam.start();
        log.lines("Started OpenCV");
    }

    @Override
    protected void START() throws InterruptedException
    {
        openCVCam.stop();
        vuforiaCam.start();
        log.lines("Started Vuforia");

        while (true)
        {
            Flow.yield();
        }
    }
}
