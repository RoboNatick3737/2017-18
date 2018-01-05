package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.opencv.OpenCVCam;

import org.firstinspires.ftc.teamcode.vision.analysis.CompetitionJewelKnocker;
import org.firstinspires.ftc.teamcode.vision.analysis.JewelDetector;

public abstract class AutonomousBase extends EnhancedOpMode implements CompetitionProgram
{
    //////     Constants for Autonomous      //////
    // How far into the start of the opmode (if we haven't moved yet) that we should jump into the main opmode regardless .
    private final long IGNORE_JEWEL_IF_NOT_VISIBLE_TIMEOUT = 10000;
    // How far we should turn to knock the ball off of the platform.
    private final double TURN_HEADING_TO_KNOCK_JEWEL = 30;


    // Instantiated and such during run progression.
    private OpenCVCam openCVCam;
    private CompetitionJewelKnocker.JewelOrder determinedJewelOrder;

    private Robot robot;

    /**
     * Where a lot of autonomous magic happens :P
     */
    @Override
    protected final void onRun() throws InterruptedException
    {
        // Initialize the robot.
        robot = new Robot(hardware);

        // We're in auto, after all.
        robot.swerveDrive.setJoystickControlEnabled(false);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        // TODO init vuforia and determine vumark

        // Wait for the jewels to be placed.
        CompetitionJewelKnocker jewelDetector = new CompetitionJewelKnocker();
        openCVCam = new OpenCVCam();
        openCVCam.start(jewelDetector, true);
        CompetitionJewelKnocker.JewelOrder currentOrder = CompetitionJewelKnocker.JewelOrder.UNKNOWN;
        while (currentOrder == CompetitionJewelKnocker.JewelOrder.UNKNOWN && !shouldTransitionIntoActualOpMode())
        {
            currentOrder = jewelDetector.getCurrentOrder();
            flow.yield();
        }
        determinedJewelOrder = currentOrder;
        openCVCam.stop();

        log.lines("Decided on " + determinedJewelOrder.toString());

        // Wait for the auto start period.
        waitForStart();

        // Knock off the jewel as quickly as possible, but skip if we couldn't tell the ball orientation.
        if (determinedJewelOrder != CompetitionJewelKnocker.JewelOrder.UNKNOWN)
        {
            double ballKnockHeading = 0;

            // Put down the knocker
            robot.ballKnocker.setKnockerTo(false);

            // Determine which direction we're going to have to rotate when auto starts.
            if (getAlliance() == Alliance.RED) // since this extends competition op mode.
            {
                if (determinedJewelOrder == CompetitionJewelKnocker.JewelOrder.BLUE_RED)
                    ballKnockHeading = TURN_HEADING_TO_KNOCK_JEWEL;
                else
                    ballKnockHeading = -TURN_HEADING_TO_KNOCK_JEWEL;

            }
            else if (getAlliance() == Alliance.BLUE)
            {
                if (determinedJewelOrder == CompetitionJewelKnocker.JewelOrder.BLUE_RED)
                    ballKnockHeading = -TURN_HEADING_TO_KNOCK_JEWEL;
                else
                    ballKnockHeading = TURN_HEADING_TO_KNOCK_JEWEL;
            }

            // Turn to that heading
            robot.swerveDrive.setDesiredHeading(ballKnockHeading);
            log.lines("Set to " + ballKnockHeading);

            // Put the knocker back up
            robot.ballKnocker.setKnockerTo(false);

            while (Math.abs(robot.gyro.z() - 30) > 5)
                robot.swerveDrive.synchronousUpdate();
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
