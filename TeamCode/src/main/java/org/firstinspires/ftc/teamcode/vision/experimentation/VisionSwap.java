package org.firstinspires.ftc.teamcode.vision.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.vuforia.VuforiaCam;
import hankutanku.EnhancedOpMode;

@Autonomous(name="Vision Swap", group= OpModeDisplayGroups.VISION_TESTING)
public class VisionSwap extends EnhancedOpMode
{
    private OpenCVCam openCVCam;
    private VuforiaCam vuforiaCam;

    @Override
    protected void onRun() throws InterruptedException
    {
        openCVCam = new OpenCVCam();
        vuforiaCam = new VuforiaCam();

        openCVCam.start(openCVCam);
        log.lines("Started OpenCV");

        waitForStart();

        openCVCam.stop();
        vuforiaCam.start();
        log.lines("Started Vuforia");

        while (true)
            flow.yield();
    }
}
