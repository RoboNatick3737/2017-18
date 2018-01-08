package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankextensions.EnhancedOpMode;
import hankextensions.structs.Vector2D;
import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.vuforia.VuforiaCam;

import org.firstinspires.ftc.teamcode.vision.analysis.BalancePlateJewelVision;
import org.firstinspires.ftc.teamcode.vision.analysis.CryptoboxTrackerBasic;

public abstract class AutonomousBase extends EnhancedOpMode implements CompetitionProgram
{
    //////     Constants for Autonomous      //////
    // How far into the start of the opmode (if we haven't moved yet) that we should jump into the main opmode regardless .
    private final long IGNORE_VISION_TARGETS_IF_NOT_VISIBLE = 10000;
    // How far we should turn to knock the ball off of the platform.
    private final double TURN_HEADING_TO_KNOCK_JEWEL = 45;


    // Instantiated and such during run progression.
    private OpenCVCam openCVCam;
    private BalancePlateJewelVision.JewelOrder determinedJewelOrder;

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

        // Determie the VuMark for the glyph placement.
//        VuforiaCam vuforiaCam = new VuforiaCam();
//        vuforiaCam.start(true);
//        VuforiaTrackable relicTemplate = vuforiaCam.getTrackables().get(0);
//        vuforiaCam.getTrackables().activate();
//        RelicRecoveryVuMark vumark = RelicRecoveryVuMark.UNKNOWN;
//        while (vumark == RelicRecoveryVuMark.UNKNOWN && !shouldTransitionIntoActualOpMode())
//        {
//            vumark = RelicRecoveryVuMark.from(relicTemplate);
//            flow.yield();
//        }
//        vuforiaCam.stop();
//        log.lines("VuMark: " + vumark.toString());

        // Wait for the auto start period.
        waitForStart();

        // Determine the jewel position
        BalancePlateJewelVision jewelDetector = new BalancePlateJewelVision();
        openCVCam = new OpenCVCam();
        openCVCam.start(jewelDetector, true);
        BalancePlateJewelVision.JewelOrder currentOrder = BalancePlateJewelVision.JewelOrder.UNKNOWN;
        long startJewelSearch = System.currentTimeMillis();
        while (currentOrder == BalancePlateJewelVision.JewelOrder.UNKNOWN && System.currentTimeMillis() - startJewelSearch < 5000)
        {
            currentOrder = jewelDetector.getCurrentOrder();
            flow.yield();
        }
        determinedJewelOrder = currentOrder;
        openCVCam.stop();
        log.lines("Jewel order: " + determinedJewelOrder.toString());

        // Knock off the jewel as quickly as possible, but skip if we couldn't tell the ball orientation.
        if (determinedJewelOrder != BalancePlateJewelVision.JewelOrder.UNKNOWN)
        {
            double ballKnockHeading = 0;

            // Put down the knocker
            robot.ballKnocker.setKnockerTo(false);

            // Determine which direction we're going to have to rotate when auto starts.
            if (getAlliance() == Alliance.RED) // since this extends competition op mode.
            {
                if (determinedJewelOrder == BalancePlateJewelVision.JewelOrder.BLUE_RED)
                    ballKnockHeading = TURN_HEADING_TO_KNOCK_JEWEL;
                else
                    ballKnockHeading = 360 - TURN_HEADING_TO_KNOCK_JEWEL;

            }
            else if (getAlliance() == Alliance.BLUE)
            {
                if (determinedJewelOrder == BalancePlateJewelVision.JewelOrder.BLUE_RED)
                    ballKnockHeading = 360 - TURN_HEADING_TO_KNOCK_JEWEL;
                else
                    ballKnockHeading = TURN_HEADING_TO_KNOCK_JEWEL;
            }

            // Turn to that heading
            robot.swerveDrive.setDesiredHeading(ballKnockHeading);
            log.lines("Turning to " + ballKnockHeading);

            while (Math.abs(robot.gyro.z() - ballKnockHeading) > 10)
            {
                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }

            // Put the knocker back up
            robot.ballKnocker.setKnockerTo(true);
        }

//        // Init the cryptobox viewer
//        CryptoboxTrackerBasic tracker = new CryptoboxTrackerBasic();
//        openCVCam.start(tracker, true);
//
//        // Push the glyph toward the flipper
//        robot.intake.intake();
//
//        // Drive off of the balance board until we see the first deposit region.
//        robot.swerveDrive.setDesiredMovement(Vector2D.rectangular(0, 0.5));
//        robot.swerveDrive.setDesiredHeading(0);
//        CryptoboxTrackerBasic.CryptoColumnPixelLocation[] locations = {};
//        while(locations.length < 2)
//        {
//            locations = tracker.getObservedLocations();
//            robot.swerveDrive.synchronousUpdate();
//            flow.yield();
//        }
//
//      // TODO Now creep side to side to the center of the column.

//        // Drive off the balance board
//        Vector2D driveDirection = Vector2D.ZERO;
//        if (getAlliance() == Alliance.RED)
//            driveDirection = Vector2D.rectangular(-0.1, -0.5);
//        else
//            driveDirection = Vector2D.rectangular(-0.1, 0.5);
//
//        robot.swerveDrive.setDesiredMovement(driveDirection);
//        robot.swerveDrive.setDesiredHeading(0);
//        robot.intake.intake(); // push glyph toward end
//
//        long startTime = System.currentTimeMillis();
//        while (System.currentTimeMillis() - startTime < 2500)
//        {
//            robot.swerveDrive.synchronousUpdate();
//            flow.yield();
//        }
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

        return System.currentTimeMillis() - opModeStartTime > IGNORE_VISION_TARGETS_IF_NOT_VISIBLE;
    }
}
