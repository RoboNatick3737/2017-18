package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.math.Vector2D;
import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.vuforia.VuforiaCam;

import org.firstinspires.ftc.teamcode.robot.hardware.SwomniModule;
import hankutanku.math.Function;
import hankutanku.math.TimedFunction;
import hankutanku.math.ParametrizedVector;
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
        final Robot robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS);
        robot.swomniDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        // Init the viewers.
        OpenCVCam openCVCam = new OpenCVCam();
        VuforiaCam vuforiaCam = new VuforiaCam();
        JewelDetector jewelDetector = new JewelDetector();
        HarvesterGlyphChecker glyphChecker = new HarvesterGlyphChecker();

        // Disable PID on driving because we want quick movements.
        for (SwomniModule module : robot.swomniDrive.swomniModules)
            module.setEnableDrivePID(false);

        // Braking helps the modules from sliding off the balance board during the match.
        for (SwomniModule module : robot.swomniDrive.swomniModules)
            module.driveMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Orient for turning
        robot.swomniDrive.orientSwerveModules(Vector2D.polar(1, 90), 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 3), flow);

        // region Initialization Detection of the Crypto Key and the Jewel Alignment

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

        // Return to default mode to drive off the platform.
        for (SwomniModule module : robot.swomniDrive.swomniModules)
            module.driveMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // region Place Pre-Loaded Glyph
        robot.intake.intake();

        // Define this so that all angles are easy to correct for the top plate.
        final double depositAngle = getBalancePlate() == BalancePlate.TOP ? getAlliance() == Alliance.BLUE ? 270 : 90 : 0;

        // Define where to place glyph
        double[] DEPOSIT_LOCATIONS;
        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            // Define locations directly off the balance board, since this auto is fairly simple.
            DEPOSIT_LOCATIONS = new double[]{61.2, 79.2, 97.8};
        }
        else
        {
            // First move off the balance board.
            robot.swomniDrive.purePursuit(ParametrizedVector.polar(
                    new Function() {
                        @Override
                        public double value(double input) {
                            return input * 15;
                        }
                    },
                    new Function() {
                        @Override
                        public double value(double input) {
                            return getAlliance() == Alliance.RED ? 270 : 90;
                        }
                    }),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 3),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 1),
                    .1,
                    null, flow);

            // Now turn to the heading which faces the cryptobox.
            robot.swomniDrive.turnRobotToHeading(depositAngle, 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 9), flow);

            DEPOSIT_LOCATIONS = new double[]{21.2, 39.2, 57.8};
        }

        // Choose the length to drive.
        final double desiredDriveLength;
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

                default: // satisfy android studio
                    desiredDriveLength = 0;
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

                default:
                    desiredDriveLength = 0;
                    break;
            }
        }

        // Drive that length slowing down over time.
        robot.swomniDrive.purePursuit(ParametrizedVector.polar(
                new Function() {
                    @Override
                    public double value(double input) {
                        return input * desiredDriveLength;
                    }
                },
                new Function() {
                    @Override
                    public double value(double input) {
                        if (getBalancePlate() == BalancePlate.BOTTOM)
                            return getAlliance() == Alliance.BLUE ? 90 : 270;
                        else
                            return 0;
                    }
                }),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 5),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 1),
                2,
                null, flow);

        // Align wheels in preparation to drive backward (this is robot-centric).
        robot.swomniDrive.orientSwerveModules(
                Vector2D.polar(1, 180),
                10,
                new TimeMeasure(TimeMeasure.Units.SECONDS, 1.5),
                flow);

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

                robot.swomniDrive.setDesiredMovement(
                        Vector2D.polar(
                                0.2 + (1 - batteryCoefficient) * .05
                                        - .15 * (closeThreshold - rangeSensorDist) / (255 - closeThreshold),
                                Vector2D.clampAngle(180 + depositAngle)));
                robot.swomniDrive.synchronousUpdate();

                flow.yield();
            }

            robot.swomniDrive.stop();
        }
        else
        {
            robot.swomniDrive.purePursuit(ParametrizedVector.polar(
                    new Function() {
                        @Override
                        public double value(double input) {
                            return input * 12.5;
                        }
                    },
                    new Function() {
                        @Override
                        public double value(double input) {
                            return Vector2D.clampAngle(180 + depositAngle); // opposite direction from angle offset (0 for bottom plate)
                        }
                    }),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 5),
                    new TimeMeasure(TimeMeasure.Units.SECONDS, 1),
                    .1, null, flow);
        }

        // Turn for better glyph placement
        double glyphPlacementAngle = 30 * (getAlliance() == Alliance.BLUE ? -1 : 1);
        robot.swomniDrive.turnRobotToHeading(
                Vector2D.clampAngle(depositAngle + glyphPlacementAngle),
                5, new TimeMeasure(TimeMeasure.Units.SECONDS, 3),
                flow);

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
        robot.swomniDrive.setDesiredHeading(depositAngle);
        double driveOffsetAngle = 10 * (getAlliance() == Alliance.BLUE ? 1 : -1);
        robot.swomniDrive.driveTime(
                ParametrizedVector.from(Vector2D.polar(0.3, Vector2D.clampAngle(depositAngle + driveOffsetAngle))),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 1.2), null, flow);

        // Smush in dat glyph
        double smushAngle = 20 * (getAlliance() == Alliance.BLUE ? 1 : -1);
        robot.swomniDrive.setDesiredHeading(Vector2D.clampAngle(depositAngle + smushAngle));
        robot.swomniDrive.driveTime(
                ParametrizedVector.from(Vector2D.polar(0.5, Vector2D.clampAngle(180 + depositAngle))),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 1.4), null, flow);

        // Make sure we aren't touching the glyph
        robot.swomniDrive.purePursuit(ParametrizedVector.polar(
                new Function() {
                    @Override
                    public double value(double input) {
                        return input * 7;
                    }
                },
                new Function() {
                    @Override
                    public double value(double input) {
                        return depositAngle;
                    }
                }),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 2),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 2),
                .1, null, flow);

        // endregion

        // Multiglyph is unreliable atm
        if (true)
            return;
        // endregion
    }
}
