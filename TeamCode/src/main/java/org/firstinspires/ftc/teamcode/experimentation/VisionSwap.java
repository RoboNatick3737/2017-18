package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.vision.OpenCVCam;
import hankextensions.vision.VuforiaCam;
import hankextensions.Core;
import hankextensions.threading.Flow;

@Autonomous(name="Vision Swap", group="Experimentation")
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
    }

    @Override
    protected void START() throws InterruptedException
    {
        openCVCam.stop();
        vuforiaCam.start();

        while (true)
        {
            Flow.yield();
        }
    }
}
