package org.firstinspires.ftc.teamcode.opmodes.experimentation.vision;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;
import org.firstinspires.ftc.teamcode.vision.JewelDetector;

@Autonomous(name="Jewel Vision", group= Constants.EXPERIMENTATION)
public class JewelVision extends EnhancedOpMode
{
    private OpenCVCam openCVCam;

    private ProcessConsole cameraProcessConsole;

    /**
     * This complex class takes care of
     */
    private JewelDetector jewelDetector;

    @Override
    protected void onRun() throws InterruptedException
    {
        // Start the good old OpenCV camera.
        openCVCam = new OpenCVCam();
        jewelDetector = new JewelDetector();
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
