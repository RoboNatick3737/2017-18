package org.firstinspires.ftc.teamcode.robot;

import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.opencv.analysis.JewelDetector;

public abstract class AutonomousBase extends RobotBase
{
    private OpenCVCam openCVCam;
    private JewelDetector.JewelOrder determinedJewelOrder;

    // How far into the start of the opmode (if we haven't moved yet) that we should jump into the main opmode.
    private final long IGNORE_JEWEL_IF_NOT_VISIBLE_TIMEOUT = 10000;

    /**
     * Where a lot of autonomous magic happens :P
     */
    @Override
    protected final void onRunWithHardware() throws InterruptedException
    {
        // We're in auto, after all.
        swerveDrive.setJoystickControlEnabled(false);

        // TODO init vuforia and determine vumark

        // Wait for the jewels to be placed.
        JewelDetector jewelDetector = new JewelDetector();
        openCVCam = new OpenCVCam();
        openCVCam.start(jewelDetector);
        JewelDetector.JewelOrder currentOrder = JewelDetector.JewelOrder.UNKNOWN;
        while (currentOrder == JewelDetector.JewelOrder.UNKNOWN && !shouldTransitionIntoActualOpMode())
        {
            currentOrder = jewelDetector.getCurrentOrder();
            flow.yield();
        }
        determinedJewelOrder = currentOrder;

        // Determine which direction we're going to have to rotate when auto starts.
        double ballKnockHeading = 0;
        if (getAlliance() == Alliance.RED) // since this extends competition op mode.
        {

        }

        // Wait for the auto start period.
        waitForStart();

        // Knock off the jewel as quickly as possible
        ballKnocker.setKnockerTo(false);


        // Run the auto itself.
        onRunAutonomous();
    }

    /**
     * Tells us when we need to just ignore both jewels or the vumark (it's taking too long)
     */
    private long opModeStartTime = -1;
    private boolean shouldTransitionIntoActualOpMode()
    {
        if (!isStarted())
            return false;

        if (opModeStartTime == -1)
            opModeStartTime = System.currentTimeMillis();

        return System.currentTimeMillis() - opModeStartTime > IGNORE_JEWEL_IF_NOT_VISIBLE_TIMEOUT;
    }

    /**
     * For when auto runs.
     */
    protected abstract void onRunAutonomous() throws InterruptedException;
}
