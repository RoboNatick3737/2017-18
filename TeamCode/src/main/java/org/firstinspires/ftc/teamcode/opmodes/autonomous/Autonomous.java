package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

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

import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.ScheduledTaskPackage;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.activity.HankuTankuRobotMonitor;
import hankutanku.math.Function;
import hankutanku.math.ParametrizedVector;
import hankutanku.math.SingleParameterRunnable;
import hankutanku.math.TimedFunction;
import hankutanku.math.Vector2D;
import hankutanku.music.Tunes;
import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.vuforia.VuforiaCam;

import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.robot.hardware.SwomniDrive;
import org.firstinspires.ftc.teamcode.robot.hardware.SwomniModule;
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
        HankuTankuRobotMonitor.gotDisconnect = false;

        // Init the bot.
        final Robot robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS);
        robot.swomniDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);
        robot.swomniDrive.setSwomniControlMode(SwomniDrive.SwomniControlMode.SWERVE_DRIVE);

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
            {
                if (jewelOrder != newJewelOrder)
                {
                    switch (newJewelOrder)
                    {
                        case BLUE_RED:
                            Tunes.play(getAlliance() == Alliance.RED ? Tunes.Option.LEFT_RIGHT_JEWEL : Tunes.Option.RIGHT_LEFT_JEWEL);
                            break;

                        case RED_BLUE:
                            Tunes.play(getAlliance() == Alliance.RED ? Tunes.Option.RIGHT_LEFT_JEWEL : Tunes.Option.LEFT_RIGHT_JEWEL);
                            break;
                    }

                    flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 3));
                }

                jewelOrder = newJewelOrder;
            }

            observedConsole.write("Currently seeing jewels: " + jewelOrder.toString() + " and vumark: " + vumark.toString());

            start = System.currentTimeMillis();

            if (isStarted() && vumark != RelicRecoveryVuMark.UNKNOWN)
                break;

            // VuMark detection.
            RelicRecoveryVuMark newVuMark = RelicRecoveryVuMark.UNKNOWN;
            vuforiaCam.start();
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
            {
                if (vumark != newVuMark)
                {
                    switch (newVuMark)
                    {
                        case LEFT:
                            Tunes.play(Tunes.Option.LEFT_COL);
                            break;
                        case CENTER:
                            Tunes.play(Tunes.Option.CENTER_COL);
                            break;
                        case RIGHT:
                            Tunes.play(Tunes.Option.RIGHT_COL);
                            break;
                    }

                    flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 2));
                }

                vumark = newVuMark;
            }

            observedConsole.write("Currently seeing jewels: " + jewelOrder.toString() + " and vumark: " + vumark.toString());

            flow.yield();
        }
        observedConsole.destroy();

        // default vumark if none detected.
        if (vumark == RelicRecoveryVuMark.UNKNOWN)
            vumark = RelicRecoveryVuMark.CENTER;

        // Anti-drift measures (any movement thus far is drift).
//        robot.gyro.startAntiDrift();
        robot.gyro.zero();

        // endregion

        // region Knock Ball
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

        // Return to default mode to drive off the platform.
        for (SwomniModule module : robot.swomniDrive.swomniModules)
            module.driveMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // region Place Pre-Loaded Glyph
        // Set based on plate.
        double[] DEPOSIT_LOCATIONS;

        // Define this so that all angles are easy to correct for the top plate.
        final double depositAngle = getBalancePlate() == BalancePlate.TOP ? getAlliance() == Alliance.BLUE ? 270 : 90 : 0;

        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            // Define locations directly off the balance board, since this auto is fairly simple.
            DEPOSIT_LOCATIONS = new double[]{60.1, 79.2, 94.7};
        }
        else
        {
            // First move off the balance board.
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

            // Now turn to the heading which faces the cryptobox.
            robot.swomniDrive.turnRobotToHeading(depositAngle, .009 + (1 - batteryCoefficient) * .05, 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 9), flow);

            DEPOSIT_LOCATIONS = new double[]{5, 21.2, 44.3};
        }

        // battery adjustment
//        double batteryDriveCorrection = batteryCoefficient * -.2;
//        for (int i = 0; i < DEPOSIT_LOCATIONS.length; i++)
//            DEPOSIT_LOCATIONS[i] += batteryDriveCorrection;

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
        robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                new Function() {
                    @Override
                    public double value(double input) {
                        return 0.28 + (1 - batteryCoefficient) * .05 - .12 * input;
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
                desiredDriveLength, null, flow);

        // Grip glyph
        robot.flipper.advanceStage(1);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 500)
            robot.flipper.update();

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
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                    new Function() {
                        @Override
                        public double value(double input) {
                            return 0.4 - .15 * input;
                        }
                    },
                    new Function() {
                        @Override
                        public double value(double input) {
                            return Vector2D.clampAngle(180 + depositAngle); // opposite direction from angle offset (0 for bottom plate)
                        }
                    }),
                    12.5, null, flow);
        }

        // Turn for better glyph placement
        double glyphPlacementAngle = 30 * (getAlliance() == Alliance.BLUE ? -1 : 1);
        robot.swomniDrive.turnRobotToHeading(
                Vector2D.clampAngle(depositAngle + glyphPlacementAngle),
                .009 + (1 - batteryCoefficient) * .05, 5, new TimeMeasure(TimeMeasure.Units.SECONDS, 8),
                flow);

        // Dump glyph
        robot.flipper.advanceStage(2);
        robot.flipper.glyphClamp.setPosition(0);

        // Drive away from glyph
        robot.swomniDrive.setDesiredHeading(depositAngle);
        double driveOffsetAngle = 10 * (getAlliance() == Alliance.BLUE ? 1 : -1);
        robot.swomniDrive.driveTime(Vector2D.polar(0.3, Vector2D.clampAngle(depositAngle + driveOffsetAngle)), 1200, flow);

        // Smush in dat glyph
        double smushAngle = 20 * (getAlliance() == Alliance.BLUE ? 1 : -1);
        robot.swomniDrive.setDesiredHeading(Vector2D.clampAngle(depositAngle + smushAngle));
        robot.swomniDrive.driveTime(Vector2D.polar(0.35, Vector2D.clampAngle(180 + depositAngle)), 1200, flow);

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
                        return depositAngle;
                    }
                }),
                7, null, flow);

        // endregion

        flow.yield();
    }
}