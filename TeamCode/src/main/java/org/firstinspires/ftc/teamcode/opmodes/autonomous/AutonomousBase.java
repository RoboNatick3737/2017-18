package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankextensions.EnhancedOpMode;
import hankextensions.structs.Vector2D;
import hankextensions.vision.opencv.OpenCVCam;

import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.structs.ComplexFunction;
import org.firstinspires.ftc.teamcode.structs.Linear;
import org.firstinspires.ftc.teamcode.structs.TimedFunction;
import org.firstinspires.ftc.teamcode.structs.VariableVector2D;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.CVCryptoKeyDetector;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.JewelAndCryptoKeyTracker;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.JewelDetector;

public abstract class AutonomousBase extends EnhancedOpMode implements CompetitionProgram
{
    private final double[] DEPOSIT_LOCATIONS = {48.5, 55, 70};

    /**
     * So here's the strat (doesn't really vary based on the autonomous).
     *
     * =========== INIT ==============
     * Detecting jewels and the crypto key during autonomous wastes precious time.  So, a single
     * OpenCV pipeline runs during the initialization phase, constantly updating the observed
     * jewel order and the observed crypto key.  Since it almost always takes a while to get
     * to init from the start of auto, this takes advantage of that extra time.
     *
     * =========== AUTO ==============
     * We already know the crypto key, so we quickly drop the jewel knocker and knock the correct
     * ball.  Then, we immediately transition into placing the glyph, depending on what we observed
     * pre-match.  Then we start multi-glyph (we probably have around 25 seconds left ideally).
     */
    @Override
    protected final void onRun() throws InterruptedException
    {
        // Init the bot.
        Robot robot = new Robot(hardware, Robot.ControlMode.AUTONOMOUS);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        // Put down the flipper glyph holder servo so that we can see the jewels.
        robot.flipper.setGlyphHolderUpTo(false);

        // region Initialization Detection of the Crypto Key and the Jewel Alignment
        JewelAndCryptoKeyTracker initializationObserver = new JewelAndCryptoKeyTracker();
        OpenCVCam cam = new OpenCVCam();
        cam.start(initializationObserver);

        CVCryptoKeyDetector.DetectedKey detectedKey = CVCryptoKeyDetector.DetectedKey.LEFT;
        JewelDetector.JewelOrder jewelOrder = JewelDetector.JewelOrder.UNKNOWN;
        while (!isStarted()) // Runs until OpMode is started, then just goes from there.
        {
            JewelDetector.JewelOrder currentOrder = initializationObserver.jewelDetector.getCurrentOrder();
            if (currentOrder != JewelDetector.JewelOrder.UNKNOWN) // don't override valid value with unknown
                jewelOrder = currentOrder;

//            CVCryptoKeyDetector.DetectedKey currentKey = initializationObserver.keyDetector.getLastDetected();
//            if (currentKey != CVCryptoKeyDetector.DetectedKey.UNKNOWN)
//                detectedKey = currentKey;

            flow.yield();
        }
        cam.stop();
        // endregion

        // region Knock Ball
        if (jewelOrder != JewelDetector.JewelOrder.UNKNOWN)
        {
            // Determine which direction we're going to have to rotate when auto starts.
            if (getAlliance() == Alliance.RED) // since this extends competition op mode.
            {
                if (jewelOrder == JewelDetector.JewelOrder.BLUE_RED)
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.RIGHT, flow);
                else
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.LEFT, flow);

            }
            else if (getAlliance() == Alliance.BLUE)
            {
                if (jewelOrder == JewelDetector.JewelOrder.BLUE_RED)
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.LEFT, flow);
                else
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.RIGHT, flow);
            }
        }
        // endregion

        // region Place Pre-Loaded Glyph

        robot.intake.intake();

        // Simple Autonomous
        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            // Choose the length to drive.
            double desiredDriveLength = 0;
            if (getAlliance() == Alliance.BLUE)
            {
                switch (detectedKey)
                {
                    case LEFT:
                        desiredDriveLength = DEPOSIT_LOCATIONS[0];
                        break;

                    case CENTER:
                        desiredDriveLength = DEPOSIT_LOCATIONS[1];
                        break;

                    case RIGHT:
                        desiredDriveLength = DEPOSIT_LOCATIONS[2];
                        break;
                }
            }
            else
            {
                switch (detectedKey)
                {
                    case LEFT:
                        desiredDriveLength = DEPOSIT_LOCATIONS[2];
                        break;

                    case CENTER:
                        desiredDriveLength = DEPOSIT_LOCATIONS[1];
                        break;

                    case RIGHT:
                        desiredDriveLength = DEPOSIT_LOCATIONS[0];
                        break;
                }
            }

            // Drive that length slowing down over time.
            VariableVector2D driveInstruction = VariableVector2D.polar(
                    new ComplexFunction(ComplexFunction.FunctionType.POLYNOMIAL, -.25 / (desiredDriveLength), 0.3),
                    new ComplexFunction(ComplexFunction.FunctionType.POLYNOMIAL, getAlliance() == Alliance.RED ? 270 : 90));
            robot.swerveDrive.driveDistance(driveInstruction, desiredDriveLength, flow);

            // Flip glyph so it slides to bottom.
            robot.flipper.setGlyphHolderUpTo(true);

            // Use math to figure out flipper position over time.
            TimedFunction flipperPosition = new TimedFunction(new Linear(-.2, 1));

            // Drive back to the cryptobox.
            robot.swerveDrive.setDesiredMovement(Vector2D.polar(0.3, 180));
            while (robot.backRangeSensor.getForwardDist() > 20)
            {
                double newFlipperPosition = flipperPosition.value();
                // Don't accidentally drop glyph.
                if (newFlipperPosition > .5)
                    robot.flipper.setFlipperPositionManually(newFlipperPosition);

                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }

            robot.swerveDrive.stop();

            // Dump glyph
            robot.flipper.advanceStage(2);
            flow.msPause(600);

            // Shove that glyph in there.
            robot.swerveDrive.setDesiredMovement(Vector2D.polar(0.3, 0));
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 900)
            {
                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }
            robot.swerveDrive.setDesiredMovement(Vector2D.polar(0.7, 180));
            start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 900)
            {
                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }
        }

        // TODO Pain in the A** autonomous
        else if (getBalancePlate() == BalancePlate.TOP)
        {
        }
        // endregion

        // region Multi-Glyph!
        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            // Drive to the glyph pile.
            robot.swerveDrive.setDesiredMovement(Vector2D.polar(1, 0));
            while (robot.frontRangeSensor.getForwardDist() > 24)
            {
                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }
            robot.swerveDrive.stop();

            // Succ in dem glyphs
            robot.intake.intake();
            flow.msPause(5000);

            // Drive back to the cryptobox.
            while (robot.backRangeSensor.getForwardDist() > 20)
            {
                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }
        }

        // TODO Pain in the A** multiglyph
        else if (getBalancePlate() == BalancePlate.TOP)
        {
        }
        // endregion
    }
}
