package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.math.Angle;
import hankutanku.math.ParametrizedVector;
import hankutanku.math.Vector2D;
import hankutanku.math.Function;
import hankutanku.math.SingleParameterRunnable;
import hankutanku.math.TimedFunction;

import hankutanku.vision.opencv.OpenCVCam;
import hankutanku.vision.vuforia.VuforiaCam;

import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
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

        // Braking helps the modules from sliding off the balance board during the first stage of auto.
        for (SwomniModule module : robot.swomniDrive.swomniModules)
            module.driveMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Orient for turning
        robot.swomniDrive.orientSwerveModulesForRotation(10, new TimeMeasure(TimeMeasure.Units.SECONDS, 3), flow);

        // region Jewels
        openCVCam.start(jewelDetector);

        // Search for the jewels until they show up in the detector.
        ProcessConsole jewelConsole = log.newProcessConsole("Jewels");
        long start = -1;
        final long jewelsNotDetectedTimeout = 5000;
        JewelDetector.JewelOrder jewelOrder = JewelDetector.JewelOrder.UNKNOWN;
        while (true)
        {
            jewelOrder = jewelDetector.getCurrentOrder();

            if (isStarted())
            {
                if (jewelOrder != JewelDetector.JewelOrder.UNKNOWN)
                    break; // start autonomous if we can see them

                if (start == -1)
                    start = System.currentTimeMillis();

                if (System.currentTimeMillis() - start > jewelsNotDetectedTimeout)
                    break;
            }

            jewelConsole.write("Looking at " + jewelOrder.toString());
            flow.yield();
        }
        jewelConsole.destroy();
        openCVCam.stop();

        log.lines("Chose " + jewelOrder.toString());

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

        // region VuMark

        // Init while turning
        vuforiaCam.start();

        robot.swomniDrive.turnRobotToHeading(Angle.degrees(20), Angle.degrees(5), new TimeMeasure(TimeMeasure.Units.SECONDS, 4), flow);

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

        robot.swomniDrive.turnRobotToHeading(Angle.ZERO, Angle.degrees(5), new TimeMeasure(TimeMeasure.Units.SECONDS, 4), flow);

        // Return to default mode to drive off the platform.
        for (SwomniModule module : robot.swomniDrive.swomniModules)
            module.driveMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // endregion

        // region Place Pre-Loaded Glyph
        robot.intake.intake();

        // Depends on top or bottom.
        double[] DEPOSIT_LOCATIONS;

        // Define this so that all angles are easy to correct for the top plate.
        final Angle depositAngle = getBalancePlate() == BalancePlate.TOP ?
                getAlliance() == Alliance.BLUE ?
                        Angle.degrees(270) :
                        Angle.degrees(90) :
                Angle.ZERO;

        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            // Define locations directly off the balance board, since this auto is fairly simple.
            DEPOSIT_LOCATIONS = new double[]{61.2, 79.2, 97.8};
        }
        else
        {
            // First move off the balance board.
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                    new Function<Double>() {
                        @Override
                        public Double value(double input) {
                            return 0.25 + (1 - batteryCoefficient) * .05 - .15 * input;
                        }
                    },
                    new Function<Angle>() {
                        @Override
                        public Angle value(double input) {
                            return getAlliance() == Alliance.RED ? Angle.degrees(270) : Angle.degrees(90);
                        }
                    }),
                    65, null, flow);

            // Now turn to the heading which faces the cryptobox.
            robot.swomniDrive.turnRobotToHeading(depositAngle, Angle.degrees(5), new TimeMeasure(TimeMeasure.Units.SECONDS, 9), flow);

            DEPOSIT_LOCATIONS = new double[]{21.2, 39.2, 57.8};
        }

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
                new Function<Double>() {
                    @Override
                    public Double value(double input) {
                        return 0.25 + (1 - batteryCoefficient) * .05 - .15 * input;
                    }
                },
                new Function<Angle>() {
                    @Override
                    public Angle value(double input) {
                        if (getBalancePlate() == BalancePlate.BOTTOM)
                            return getAlliance() == Alliance.BLUE ? Angle.degrees(90) : Angle.degrees(270);
                        else
                            return Angle.ZERO;
                    }
                }),
                desiredDriveLength, null, flow);

        // Align wheels in preparation to drive backward (this is robot-centric).
        robot.swomniDrive.orientSwerveModules(
                Vector2D.polar(1, Angle.degrees(180)),
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
                                depositAngle.opposing()));
                robot.swomniDrive.synchronousUpdate();

                flow.yield();
            }

            robot.swomniDrive.stop();
        }
        else
        {
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                    new Function<Double>() {
                        @Override
                        public Double value(double input) {
                            return 0.4 - .3 * input;
                        }
                    },
                    new Function<Angle>() {
                        @Override
                        public Angle value(double input) {
                            return depositAngle.opposing();
                        }
                    }),
                    12.5, null, flow);
        }

        // Turn for better glyph placement
        Angle glyphPlacementAngle = Angle.degrees(30 * (getAlliance() == Alliance.BLUE ? -1 : 1));
        robot.swomniDrive.turnRobotToHeading(depositAngle.add(glyphPlacementAngle), Angle.degrees(5), new TimeMeasure(TimeMeasure.Units.SECONDS, 3), flow);

        // Dump glyph
        TimedFunction<Double> flipperPos = new TimedFunction<>(new Function<Double>() {
            @Override
            public Double value(double input) {
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
        Angle driveOffsetAngle = Angle.degrees(10 * (getAlliance() == Alliance.BLUE ? 1 : -1));
        robot.swomniDrive.driveTime(Vector2D.polar(0.3, depositAngle.add(driveOffsetAngle)), new TimeMeasure(TimeMeasure.Units.SECONDS, 1.2), flow);

        // Smush in dat glyph
        Angle smushAngle = Angle.degrees(20 * (getAlliance() == Alliance.BLUE ? 1 : -1));
        robot.swomniDrive.setDesiredHeading(depositAngle.add(smushAngle));
        robot.swomniDrive.driveTime(Vector2D.polar(0.5, depositAngle.opposing()), new TimeMeasure(TimeMeasure.Units.SECONDS, 1.4), flow);

        // Make sure we aren't touching the glyph
        robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                new Function<Double>() {
                    @Override
                    public Double value(double input) {
                        return 0.3;
                    }
                },
                new Function<Angle>() {
                    @Override
                    public Angle value(double input) {
                        return depositAngle;
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
            robot.swomniDrive.setDesiredHeading(Angle.ZERO);
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                    new Function<Double>() {
                        @Override
                        public Double value(double input) {
                            return 0.6 - input * 0.3;
                        }
                    },
                    new Function<Angle>() {
                        @Override
                        public Angle value(double input) {
                            return Angle.ZERO;
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
            robot.swomniDrive.setDesiredMovement(Vector2D.polar(0.3, Angle.ZERO));
            while (glyphChecker.getGlyphsHarvested() < 2)
            {
                robot.swomniDrive.synchronousUpdate();
                flow.yield();
            }

            // Drive back.
            robot.swomniDrive.driveDistance(ParametrizedVector.polar(
                    new Function<Double>() {
                        @Override
                        public Double value(double input) {
                            return -0.6 + input * 0.3;
                        }
                    },
                    new Function<Angle>() {
                        @Override
                        public Angle value(double input) {
                            return Angle.ZERO;
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
