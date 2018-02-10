package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankextensions.EnhancedOpMode;
import hankextensions.structs.Vector2D;
import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.vuforia.VuforiaCam;

import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.structs.Function;
import org.firstinspires.ftc.teamcode.structs.Polynomial;
import org.firstinspires.ftc.teamcode.structs.SingleParameterRunnable;
import org.firstinspires.ftc.teamcode.structs.TimedFunction;
import org.firstinspires.ftc.teamcode.structs.ParametrizedVector;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.HarvesterGlyphChecker;
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
        // Init the bot.
        final Robot robot = new Robot(hardware, Robot.ControlMode.AUTONOMOUS);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        // Init the viewers.
        JewelDetector jewelDetector = new JewelDetector();
        HarvesterGlyphChecker glyphChecker = new HarvesterGlyphChecker();

        // Align wheels sideways to drive off the platform.
        robot.swerveDrive.orientSwerveModules(Vector2D.polar(1, 90), 15, 3000, flow);

        // Put down the flipper glyph holder servo so that we can see the jewels.
        robot.flipper.setGlyphHolderUpTo(false);

        // region Initialization Detection of the Crypto Key and the Jewel Alignment
        OpenCVCam openCVCam = new OpenCVCam();
        VuforiaCam vuforiaCam = new VuforiaCam();

        // DON'T specify a default order, if we mess this up we lose points.
        JewelDetector.JewelOrder jewelOrder = JewelDetector.JewelOrder.UNKNOWN;
        RelicRecoveryVuMark vumark = RelicRecoveryVuMark.UNKNOWN;

        // Loop through
        ProcessConsole observedConsole = log.newProcessConsole("Observed Init stuff");
        while (!isStarted()) // Runs until OpMode is started, then just goes from there.
        {
            // Jewel detection
            openCVCam.start(jewelDetector);
            JewelDetector.JewelOrder newJewelOrder = JewelDetector.JewelOrder.UNKNOWN;
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 5000 && newJewelOrder == JewelDetector.JewelOrder.UNKNOWN)
            {
                newJewelOrder = jewelDetector.getCurrentOrder();

                if (isStarted() && jewelOrder != JewelDetector.JewelOrder.UNKNOWN)
                    break;

                flow.yield();
            }
            openCVCam.stop();

            if (newJewelOrder != JewelDetector.JewelOrder.UNKNOWN)
                jewelOrder = newJewelOrder;

            observedConsole.write("Currently seeing jewels: " + jewelOrder.toString() + " and vumark: " + vumark.toString());

            start = System.currentTimeMillis();

            if (isStarted() && vumark != RelicRecoveryVuMark.UNKNOWN)
                break;

            // VuMark detection.
            RelicRecoveryVuMark newVuMark = RelicRecoveryVuMark.UNKNOWN;
            vuforiaCam.start(true);
            VuforiaTrackable relicTemplate = vuforiaCam.getTrackables().get(0);
            vuforiaCam.getTrackables().activate();
            while (System.currentTimeMillis() - start < 10000 && newVuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                newVuMark = RelicRecoveryVuMark.from(relicTemplate);

                if (isStarted() && vumark != RelicRecoveryVuMark.UNKNOWN)
                    break;

                flow.yield();
            }
            vuforiaCam.stop(flow);

            if (newVuMark != RelicRecoveryVuMark.UNKNOWN)
                vumark = newVuMark;

            observedConsole.write("Currently seeing jewels: " + jewelOrder.toString() + " and vumark: " + vumark.toString());

            flow.yield();
        }
        observedConsole.destroy();

        // default vumark if none detected.
        if (vumark == RelicRecoveryVuMark.UNKNOWN)
            vumark = RelicRecoveryVuMark.CENTER;

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
            double[] DEPOSIT_LOCATIONS = {58.2 , 75.2, 92.8};

            // battery adjustment
            double batteryDriveCorrection = batteryCoefficient * -.2;
            for (int i = 0; i < DEPOSIT_LOCATIONS.length; i++)
                DEPOSIT_LOCATIONS[i] += batteryDriveCorrection;

            // Adjust them for the alliance
            double adjust = getAlliance() == Alliance.BLUE ? .9 : -.9; // offset on balance board
            for (int i = 0; i < DEPOSIT_LOCATIONS.length; i++)
                DEPOSIT_LOCATIONS[i] += adjust;

            // Choose the length to drive.
            double desiredDriveLength = 0;
            if (getAlliance() == Alliance.BLUE)
            {
                switch (vumark)
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
                switch (vumark)
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
            robot.swerveDrive.driveDistance(ParametrizedVector.polar(
                    new Function() {
                        @Override
                        public double value(double input) {
                            return 0.5 - .3 * input;
                        }
                    },
                    new Function() {
                        @Override
                        public double value(double input) {
                            return getAlliance() == Alliance.RED ? 270 : 90;
                        }
                    }),
                    desiredDriveLength, null, flow);

            // Flip glyph so it slides to bottom.
            robot.flipper.setGlyphHolderUpTo(true);

            // Align wheels backward.
            robot.swerveDrive.orientSwerveModules(Vector2D.polar(1, 180), 10, 1500, flow);

            // Drive back to the cryptobox.
            robot.swerveDrive.driveDistance(ParametrizedVector.polar(
                    new Function() {
                        @Override
                        public double value(double input) {
                            return 0.4 - .3 * input;
                        }
                    },
                    new Function() {
                        @Override
                        public double value(double input) {
                            return 180;
                        }
                    }),
                    15.5, null, flow);

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
            robot.intake.stop();
            robot.flipper.advanceStage(2);

            // Drive away from glyph
            robot.swerveDrive.setDesiredHeading(0);
            robot.swerveDrive.driveTime(Vector2D.polar(0.3, getAlliance() == Alliance.BLUE ? 10 : 350), 1200, flow);

            // Shove glyph in
            robot.swerveDrive.setDesiredHeading(getAlliance() == Alliance.BLUE ? 20 : 340);// A bit of rotation helps smush the cube in.
            robot.swerveDrive.driveTime(Vector2D.polar(0.5, 180), 1400, flow);
        }

        // TODO Pain in the A** autonomous
        else if (getBalancePlate() == BalancePlate.TOP)
        {
        }

        // Make sure we aren't touching the glyph
        robot.swerveDrive.driveDistance(ParametrizedVector.polar(
                new Function() {
                    @Override
                    public double value(double input) {
                        return 0.3;
                    }
                },
                new Function() {
                    @Override
                    public double value(double input) {
                        return 0;
                    }
                }),
                7, null, flow);

        // endregion

        // Multiglyph is unreliable atm
        if (true)
            return;

        // region TODO Multi-Glyph!
        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            // Start the intake.
            robot.intake.intake();

            // Drive to where the glyph pit is but stop directly in front of it.
            robot.swerveDrive.setDesiredHeading(0);
            robot.swerveDrive.driveDistance(ParametrizedVector.polar(
                    new Function() {
                        @Override
                        public double value(double input) {
                            return 0.6 - input * 0.3;
                        }
                    },
                    new Function() {
                        @Override
                        public double value(double input) {
                            return 0;
                        }
                    }),
                    40,
                    new SingleParameterRunnable() {
                        @Override
                        public void run(double param) {
                            if (param > .66)
                            {
                                // Put the flipper back down.
                                robot.flipper.advanceStage(0);
                            }
                        }
                    },
                    flow
            );

            // Initialize the viewer and wait for glyphs to show up in the harvester.
            openCVCam.start(glyphChecker);
            robot.swerveDrive.setDesiredMovement(Vector2D.polar(0.3, 0));
            while (glyphChecker.getGlyphsHarvested() < 2)
            {
                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }

            // Drive back.
            robot.swerveDrive.driveDistance(ParametrizedVector.polar(
                    new Function() {
                        @Override
                        public double value(double input) {
                            return -0.6 + input * 0.3;
                        }
                    },
                    new Function() {
                        @Override
                        public double value(double input) {
                            return 0;
                        }
                    }),
                    50,
                    null,
                    flow
            );
        }

        // TODO Pain in the A** multiglyph
        else if (getBalancePlate() == BalancePlate.TOP)
        {
        }
        // endregion
    }
}
