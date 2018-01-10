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

import org.firstinspires.ftc.teamcode.vision.analysis.JewelDetector;

public abstract class AutonomousBase extends EnhancedOpMode implements CompetitionProgram
{
    //////     Constants for Autonomous      //////
    // How far into the start of the opmode (if we haven't moved yet) that we should jump into the main opmode regardless .
    private final long IGNORE_VISION_TARGETS_IF_NOT_VISIBLE = 10000;
    // How far we should turn to knock the ball off of the platform.
    private final double TURN_HEADING_TO_KNOCK_JEWEL = 45;


    // Instantiated and such during run progression.
    private OpenCVCam openCVCam;
    private JewelDetector.JewelOrder determinedJewelOrder;

    private Robot robot;

    /**
     * Where a lot of autonomous magic happens :P
     */
    @Override
    protected final void onRun() throws InterruptedException
    {
        long start; // for timed stuff.

        // Initialize the robot.
        robot = new Robot(hardware);

        // We're in auto, after all.
        robot.swerveDrive.setJoystickControlEnabled(false);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        // Init the jewel detector (saves time)
        JewelDetector jewelDetector = new JewelDetector();
        openCVCam = new OpenCVCam();
        openCVCam.start(jewelDetector, true);

        // Wait for the auto start period.
        waitForStart();

        // Get the jewel position.
        JewelDetector.JewelOrder currentOrder = JewelDetector.JewelOrder.UNKNOWN;
        start = System.currentTimeMillis();
        while (currentOrder == JewelDetector.JewelOrder.UNKNOWN && System.currentTimeMillis() - start < 5000)
        {
            currentOrder = jewelDetector.getCurrentOrder();
            flow.yield();
        }
        determinedJewelOrder = currentOrder;
        openCVCam.stop();
        log.lines("Jewel order: " + determinedJewelOrder.toString());

        // Knock off the jewel as quickly as possible, but skip if we couldn't tell the ball orientation.
        if (determinedJewelOrder != JewelDetector.JewelOrder.UNKNOWN)
        {
            double ballKnockHeading = 0;

            // Put down the knocker
            robot.ballKnocker.setKnockerTo(false);

            // Determine which direction we're going to have to rotate when auto starts.
            if (getAlliance() == Alliance.RED) // since this extends competition op mode.
            {
                if (determinedJewelOrder == JewelDetector.JewelOrder.BLUE_RED)
                    ballKnockHeading = TURN_HEADING_TO_KNOCK_JEWEL;
                else
                    ballKnockHeading = 360 - TURN_HEADING_TO_KNOCK_JEWEL;

            }
            else if (getAlliance() == Alliance.BLUE)
            {
                if (determinedJewelOrder == JewelDetector.JewelOrder.BLUE_RED)
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

        // Drive off of the balance board.
        Vector2D desiredMovement = null;
        if (getAlliance() == Alliance.RED)
            desiredMovement = Vector2D.polar(0.5, 270);
        else
            desiredMovement = Vector2D.polar(0.5, 90);
        robot.swerveDrive.setDesiredMovement(desiredMovement);
        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 5000)
        {
            robot.swerveDrive.synchronousUpdate();
            flow.yield();
        }

        // Determine the VuMark for the glyph placement.
        VuforiaCam vuforiaCam = new VuforiaCam();
        vuforiaCam.start(true);
        VuforiaTrackable relicTemplate = vuforiaCam.getTrackables().get(0);
        vuforiaCam.getTrackables().activate();
        RelicRecoveryVuMark vumark = RelicRecoveryVuMark.UNKNOWN;
        start = System.currentTimeMillis();
        while (vumark == RelicRecoveryVuMark.UNKNOWN && System.currentTimeMillis() - start < 3000)
        {
            vumark = RelicRecoveryVuMark.from(relicTemplate);
            flow.yield();
        }
        vuforiaCam.stop();
        log.lines("VuMark: " + vumark.toString());
    }
}
