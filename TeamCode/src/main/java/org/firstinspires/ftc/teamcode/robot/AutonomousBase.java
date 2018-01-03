package org.firstinspires.ftc.teamcode.robot;

import hankextensions.structs.Vector2D;
import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.opencv.analysis.JewelDetector;

public abstract class AutonomousBase extends RobotBase
{
    //////     Constants for Autonomous      //////
    // How far into the start of the opmode (if we haven't moved yet) that we should jump into the main opmode regardless .
    private final long IGNORE_JEWEL_IF_NOT_VISIBLE_TIMEOUT = 10000;
    // How far we should turn to knock the ball off of the platform.
    private final double TURN_HEADING_TO_KNOCK_JEWEL = 30;


    // Instantiated and such during run progression.
    private OpenCVCam openCVCam;
    private JewelDetector.JewelOrder determinedJewelOrder;

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
        openCVCam.stop();

        // Wait for the auto start period.
        waitForStart();

        // Knock off the jewel as quickly as possible, but skip if we couldn't tell the ball orientation.
        double ballKnockHeading = 0;
        if (determinedJewelOrder != JewelDetector.JewelOrder.UNKNOWN)
        {
            // Put down the knocker
            ballKnocker.setKnockerTo(false);

            // Determine which direction we're going to have to rotate when auto starts.
            if (getAlliance() == Alliance.RED) // since this extends competition op mode.
            {
                if (determinedJewelOrder == JewelDetector.JewelOrder.BLUE_RED)
                    ballKnockHeading = TURN_HEADING_TO_KNOCK_JEWEL;
                else
                    ballKnockHeading = -TURN_HEADING_TO_KNOCK_JEWEL;

            }
            else if (getAlliance() == Alliance.BLUE)
            {
                if (determinedJewelOrder == JewelDetector.JewelOrder.BLUE_RED)
                    ballKnockHeading = -TURN_HEADING_TO_KNOCK_JEWEL;
                else
                    ballKnockHeading = TURN_HEADING_TO_KNOCK_JEWEL;
            }

            // Turn to that heading
            swerveDrive.setDesiredHeading(ballKnockHeading);

            // Put the knocker back up
            ballKnocker.setKnockerTo(true);
        }

        // Drive off of the balance board.
        // todo figure out driving: swerveDrive.setDesiredMovement(Vector2D.rectangular(-1, 0));
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
}
