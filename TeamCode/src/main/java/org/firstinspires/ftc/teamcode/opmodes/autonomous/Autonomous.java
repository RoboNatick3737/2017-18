package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankextensions.EnhancedOpMode;
import hankextensions.structs.Vector2D;
import hankextensions.vision.opencv.OpenCVCam;

import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.structs.Function;
import org.firstinspires.ftc.teamcode.structs.Polynomial;
import org.firstinspires.ftc.teamcode.structs.TimedFunction;
import org.firstinspires.ftc.teamcode.structs.VariableVector2D;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.CVCryptoKeyDetector;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.JewelAndCryptoKeyTracker;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.JewelDetector;

public abstract class Autonomous extends EnhancedOpMode implements CompetitionProgram
{
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
        double batteryCoefficient = getBatteryCoefficient();

        // Init the bot.
        Robot robot = new Robot(hardware, Robot.ControlMode.AUTONOMOUS);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        // Align wheels sideways to drive off the platform.
        robot.swerveDrive.orientSwerveModules(Vector2D.polar(1, 90), 15, flow);

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

            // Until cryptokey works this is for testing
            if (C1.gamepad.a)
                detectedKey = CVCryptoKeyDetector.DetectedKey.CENTER;
            else if (C1.gamepad.b)
                detectedKey = CVCryptoKeyDetector.DetectedKey.RIGHT;

            flow.yield();
        }
        cam.stop();
        log.lines("Jewels are " + jewelOrder.toString() + " and key is " + detectedKey.toString());
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

        double batteryDriveCorrection = batteryCoefficient * -.5;
        double[] DEPOSIT_LOCATIONS = {58.2 + batteryDriveCorrection , 74.7 + batteryDriveCorrection, 92.3 + batteryDriveCorrection};
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
            robot.swerveDrive.driveDistance(VariableVector2D.polar(
                    new Polynomial(-.15 / (desiredDriveLength), 0.3),
                    new Polynomial(getAlliance() == Alliance.RED ? 270 : 90)), desiredDriveLength, flow);

            // Flip glyph so it slides to bottom.
            robot.flipper.setGlyphHolderUpTo(true);

            // Align wheels backward.
            robot.swerveDrive.orientSwerveModules(Vector2D.polar(1, 180), 15, flow);

            // Drive back to the cryptobox.
            robot.swerveDrive.driveDistance(VariableVector2D.polar(
                    new Polynomial(-.12 / (20), 0.2),
                    new Polynomial(180)), 16, flow);

            // Turn for better glyph placement
            double desiredHeading = getAlliance() == Alliance.BLUE ? 330 : 30;
            robot.swerveDrive.setDesiredHeading(desiredHeading);
            while (Math.abs(robot.gyro.getHeading() - desiredHeading) > 3)
                robot.swerveDrive.synchronousUpdate();
            robot.swerveDrive.stop();

            // Dump glyph
            TimedFunction flipperPos = new TimedFunction(new Function() {
                @Override
                public double value(double input) {
                    return -.25 * input + .8;
                }
            });
            while (true)
            {
                if (flipperPos.value() < .4)
                    break;

                robot.flipper.setFlipperPositionManually(flipperPos.value());

                flow.yield();
            }
            robot.flipper.advanceStage(2);

            // Drive away from glyph
            robot.swerveDrive.setDesiredHeading(0);
            robot.swerveDrive.setDesiredMovement(Vector2D.polar(0.3, getAlliance() == Alliance.BLUE ? 10 : 350));
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 1200)
            {
                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }

            // Shove glyph in
            robot.swerveDrive.setDesiredHeading(getAlliance() == Alliance.BLUE ? 20 : 340);// A bit of rotation helps smush the cube in.
            robot.swerveDrive.setDesiredMovement(Vector2D.polar(0.5, 180));
            start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 1400)
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
            // To the glyph pit!
            robot.swerveDrive.setDesiredHeading(0);
            robot.swerveDrive.setDesiredMovement(Vector2D.polar(0.6, 0));
            long start = System.currentTimeMillis();
            boolean flipperDown = false;
            while (System.currentTimeMillis() - start < 1500)
            {
                if (!flipperDown && System.currentTimeMillis() - start > 1000)
                {
                    // Put the flipper back down.
                    robot.flipper.advanceStage(0);
                    flipperDown = true;
                }

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
