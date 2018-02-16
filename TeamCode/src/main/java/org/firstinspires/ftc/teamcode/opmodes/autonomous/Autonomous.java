package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankextensions.EnhancedOpMode;
import hankextensions.structs.Vector2D;
import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.vuforia.VuforiaCam;

import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.robot.hardware.SwomniModule;
import org.firstinspires.ftc.teamcode.structs.Function;
import org.firstinspires.ftc.teamcode.structs.SingleParameterRunnable;
import org.firstinspires.ftc.teamcode.structs.TimedFunction;
import org.firstinspires.ftc.teamcode.structs.ParametrizedVector;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.HarvesterGlyphChecker;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.JewelDetector;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.NonLocalizedJewelDetector;

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
        final Robot robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS);
        robot.swomniDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        // Init the viewers.
        JewelDetector jewelDetector = new JewelDetector();
        HarvesterGlyphChecker glyphChecker = new HarvesterGlyphChecker();

        // Put down the flipper glyph holder servo so that we can see the jewels.
//        robot.flipper.setGlyphHolderUpTo(false);

        // Disable PID on driving because we want quick movements.
        for (SwomniModule module : robot.swomniDrive.swomniModules)
            module.setEnableDrivePID(false);

        // Orient for turning
        robot.swomniDrive.orientSwerveModulesForRotation(10, 3000, flow);

        // DON'T specify a default order, if we mess this up we lose points.
        OpenCVCam openCVCam = new OpenCVCam();
        JewelDetector.JewelOrder jewelOrder = JewelDetector.JewelOrder.UNKNOWN;

        // region Jewel detection
        openCVCam.start(jewelDetector);

        ProcessConsole jewelConsole = log.newProcessConsole("Jewels");
        while (!isStarted())
        {
            jewelConsole.write("Looking at " + jewelDetector.getCurrentOrder().toString());
            flow.yield();
        }
        jewelConsole.destroy();

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 5000)
        {
            jewelOrder = jewelDetector.getCurrentOrder();

            if (jewelOrder != JewelDetector.JewelOrder.UNKNOWN)
                break;

            flow.yield();
        }
        openCVCam.stop();

        if (jewelOrder != JewelDetector.JewelOrder.UNKNOWN)
        {
            // Determine which direction we're going to have to rotate when auto starts.
            if (getAlliance() == Alliance.RED) // since this extends competition op mode.
            {
                if (jewelOrder == JewelDetector.JewelOrder.BLUE_RED)
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.LEFT, flow);
                else
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.RIGHT, flow);

            }
            else if (getAlliance() == Alliance.BLUE)
            {
                if (jewelOrder == JewelDetector.JewelOrder.BLUE_RED)
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.RIGHT, flow);
                else
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.LEFT, flow);
            }
        }
        // endregion

        // Init while turning
        VuforiaCam vuforiaCam = new VuforiaCam();
        vuforiaCam.start();

        // region Detect cryptokey
        robot.swomniDrive.turnRobotToHeading(20, 5, 4000, flow);

        RelicRecoveryVuMark detectedVuMark = RelicRecoveryVuMark.UNKNOWN;

        VuforiaTrackable relicTemplate = vuforiaCam.getTrackables().get(0);
        vuforiaCam.getTrackables().activate();
        ProcessConsole vuforiaConsole = log.newProcessConsole("Vuforia");

        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 9000)
        {
            detectedVuMark = RelicRecoveryVuMark.from(relicTemplate);
            vuforiaConsole.write("Currently seeing " + detectedVuMark.toString());

            if (detectedVuMark != RelicRecoveryVuMark.UNKNOWN)
                break;

            flow.yield();
        }

        log.lines("Chose " + detectedVuMark.toString());

        if (detectedVuMark == RelicRecoveryVuMark.UNKNOWN)
            detectedVuMark = RelicRecoveryVuMark.CENTER;

        vuforiaConsole.destroy();

        robot.swomniDrive.turnRobotToHeading(0, 5, 4000, flow);
        // endregion

        // region Place Pre-Loaded Glyph
        robot.intake.intake();

        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            double[] DEPOSIT_LOCATIONS = new double[]{61.2, 79.2, 97.8};

            // battery adjustment
            double batteryDriveCorrection = batteryCoefficient * -.2;
            for (int i = 0; i < DEPOSIT_LOCATIONS.length; i++)
                DEPOSIT_LOCATIONS[i] += batteryDriveCorrection;

            // Choose the length to drive.
            double desiredDriveLength = 0;
            if (getAlliance() == Alliance.BLUE)
            {
                switch (detectedVuMark)
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
                switch (detectedVuMark)
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
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                    new Function() {
                        @Override
                        public double value(double input) {
                            return 0.25 + (1 - batteryCoefficient) * .05 - .15 * input;
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
//            robot.flipper.setGlyphHolderUpTo(true);

            // Align wheels backward.
            robot.swomniDrive.orientSwerveModules(Vector2D.polar(1, 180), 10, 1500, flow);

            // Drive back to the cryptobox, using range sensor if possible.
            if (robot.backRangeSensor.initializedCorrectly)
            {
                int streak = 0;
                double closeThreshold = 24;
                while (true)
                {
                    double rangeSensorDist = robot.backRangeSensor.getForwardDist();

                    if (rangeSensorDist < closeThreshold)
                    {
                        streak++;

                        if (streak > 3)
                        {
                            break;
                        }
                    }
                    else
                        streak = 0;

                    robot.swomniDrive.setDesiredMovement(Vector2D.polar(0.2 + (1 - batteryCoefficient) * .05 - .15 * (closeThreshold - rangeSensorDist) / (255 - closeThreshold), 180));

                    robot.swomniDrive.synchronousUpdate();

                    flow.yield();
                }

                robot.swomniDrive.stop();
            }
            else
            {
                robot.swomniDrive.driveDistance(ParametrizedVector.polar(
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
                        12.5, null, flow);
            }

            // Turn for better glyph placement
            robot.swomniDrive.turnRobotToHeading(getAlliance() == Alliance.BLUE ? 330 : 30, 5, 3000, flow);

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
            robot.swomniDrive.setDesiredHeading(0);
            robot.swomniDrive.driveTime(Vector2D.polar(0.3, getAlliance() == Alliance.BLUE ? 10 : 350), 1200, flow);

            // Shove glyph in
            robot.swomniDrive.setDesiredHeading(getAlliance() == Alliance.BLUE ? 20 : 340);// A bit of rotation helps smush the cube in.
            robot.swomniDrive.driveTime(Vector2D.polar(0.5, 180), 1400, flow);

            // Make sure we aren't touching the glyph
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
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
        }
        else if (getBalancePlate() == BalancePlate.TOP)
        {
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                    new Function() {
                        @Override
                        public double value(double input) {
                            return 0.25 + (1 - batteryCoefficient) * .05 - .15 * input;
                        }
                    },
                    new Function() {
                        @Override
                        public double value(double input) {
                            return getAlliance() == Alliance.RED ? 270 : 90;
                        }
                    }),
                    65, null, flow);

            robot.swomniDrive.turnRobotToHeading(getAlliance() == Alliance.BLUE ? 270 : 90, 5, 9000, flow);

            double[] DEPOSIT_LOCATIONS = new double[]{21.2, 39.2, 57.8};

            // battery adjustment
            double batteryDriveCorrection = batteryCoefficient * -.2;
            for (int i = 0; i < DEPOSIT_LOCATIONS.length; i++)
                DEPOSIT_LOCATIONS[i] += batteryDriveCorrection;

            // Choose the length to drive.
            double desiredDriveLength = 0;
            if (getAlliance() == Alliance.BLUE)
            {
                switch (detectedVuMark)
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
                switch (detectedVuMark)
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
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                    new Function() {
                        @Override
                        public double value(double input) {
                            return 0.25 + (1 - batteryCoefficient) * .05 - .15 * input;
                        }
                    },
                    new Function() {
                        @Override
                        public double value(double input) {
                            return getAlliance() == Alliance.RED ? 180 : 0;
                        }
                    }),
                    desiredDriveLength, null, flow);

            // Align wheels backward.
            robot.swomniDrive.orientSwerveModules(Vector2D.polar(1, 180), 10, 1500, flow);

            // Drive back to the cryptobox, using range sensor if possible.
            if (robot.backRangeSensor.initializedCorrectly)
            {
                int streak = 0;
                double closeThreshold = 24;
                while (true)
                {
                    double rangeSensorDist = robot.backRangeSensor.getForwardDist();

                    if (rangeSensorDist < closeThreshold)
                    {
                        streak++;

                        if (streak > 3)
                        {
                            break;
                        }
                    }
                    else
                        streak = 0;

                    robot.swomniDrive.setDesiredMovement(Vector2D.polar(0.2 + (1 - batteryCoefficient) * .05 - .15 * (closeThreshold - rangeSensorDist) / (255 - closeThreshold), getAlliance() == Alliance.RED ? 270 : 90));

                    robot.swomniDrive.synchronousUpdate();

                    flow.yield();
                }

                robot.swomniDrive.stop();
            }
            else
            {
                robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                        new Function() {
                            @Override
                            public double value(double input) {
                                return 0.4 - .3 * input;
                            }
                        },
                        new Function() {
                            @Override
                            public double value(double input) {
                                return getAlliance() == Alliance.RED ? 270 : 90;
                            }
                        }),
                        12.5, null, flow);
            }

            // Turn for better glyph placement
            robot.swomniDrive.turnRobotToHeading(getAlliance() == Alliance.BLUE ? getAlliance() == Alliance.RED ? 60 : 240 : getAlliance() == Alliance.RED ? 300 : 120, 5, 3000, flow);

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

            if (true)
                return;

            // Drive away from glyph
            robot.swomniDrive.setDesiredHeading(0);
            robot.swomniDrive.driveTime(Vector2D.polar(0.3, getAlliance() == Alliance.BLUE ? 10 : 350), 1200, flow);

            // Shove glyph in
            robot.swomniDrive.setDesiredHeading(getAlliance() == Alliance.BLUE ? 20 : 340);// A bit of rotation helps smush the cube in.
            robot.swomniDrive.driveTime(Vector2D.polar(0.5, 180), 1400, flow);

            // Make sure we aren't touching the glyph
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
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
        }

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
            robot.swomniDrive.setDesiredHeading(0);
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
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
            robot.swomniDrive.setDesiredMovement(Vector2D.polar(0.3, 0));
            while (glyphChecker.getGlyphsHarvested() < 2)
            {
                robot.swomniDrive.synchronousUpdate();
                flow.yield();
            }

            // Drive back.
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
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
