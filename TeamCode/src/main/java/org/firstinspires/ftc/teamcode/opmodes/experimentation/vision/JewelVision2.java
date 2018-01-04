package org.firstinspires.ftc.teamcode.opmodes.experimentation.vision;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;
import org.firstinspires.ftc.teamcode.vision.JewelDetector2;

@Autonomous(name="Jewel Vision 2", group= Constants.EXPERIMENTATION)
public class JewelVision2 extends EnhancedOpMode
{
    private OpenCVCam openCVCam;

    private ProcessConsole cameraProcessConsole;

    /**
     * This complex class takes care of
     */
    private JewelDetector2 jewelDetector;

    @Override
    protected void onRun() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        jewelDetector = new JewelDetector2();
        openCVCam.start(jewelDetector);

        cameraProcessConsole = log.newProcessConsole("Camera Process Console");

        waitForStart();

        while (true)
        {
            cameraProcessConsole.write("Order is " + jewelDetector.getCurrentOrder().toString());
            flow.yield();
        }
    }
}
