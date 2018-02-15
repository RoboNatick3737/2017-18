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
import org.firstinspires.ftc.teamcode.structs.Function;
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
        final Robot robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS);
        robot.swomniDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        // Init the viewers.
        JewelDetector jewelDetector = new JewelDetector();
        HarvesterGlyphChecker glyphChecker = new HarvesterGlyphChecker();

        // Put down the flipper glyph holder servo so that we can see the jewels.
//        robot.flipper.setGlyphHolderUpTo(false);

        // Orient for turning
        robot.swomniDrive.orientSwerveModulesForRotation(10, 3000, flow);

        // region Detect cryptokey pose during initialization
        RelicRecoveryVuMark detectedVuMark = RelicRecoveryVuMark.UNKNOWN;
        double angleOffset = 0;

        VuforiaCam vuforiaCam = new VuforiaCam();
        vuforiaCam.start();
        VuforiaTrackable relicTemplate = vuforiaCam.getTrackables().get(0);
        vuforiaCam.getTrackables().activate();
        ProcessConsole vuforiaConsole = log.newProcessConsole("Vuforia");
        while (!isStarted())
        {
            detectedVuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (detectedVuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                if (pose != null) {
//                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

//                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
//                    double tX = trans.get(0);
//                    double tY = trans.get(1);
//                    double tZ = trans.get(2);
//
//                    // Extract the rotational components of the target relative to the robot
//                    double rX = rot.firstAngle;
//                    double rY = rot.secondAngle;
//                    double rZ = rot.thirdAngle;


                    vuforiaConsole.write(
                            "Detected: " + detectedVuMark.toString(),
                            "Rotation: <" + rot.firstAngle + ", " + rot.secondAngle + ", " + rot.thirdAngle + ">");

                    angleOffset = rot.secondAngle;
                }
                else
                {
                    vuforiaConsole.write("Detected: " + detectedVuMark.toString());
                }
            }

            flow.yield();
        }
        vuforiaConsole.destroy();
        // endregion

        // Turn for angle offset.
        robot.gyro.applyOffset(angleOffset);
        robot.swomniDrive.turnRobotToHeading(0, 2, 5000, flow);

        // DON'T specify a default order, if we mess this up we lose points.
        OpenCVCam openCVCam = new OpenCVCam();
        JewelDetector.JewelOrder jewelOrder = JewelDetector.JewelOrder.UNKNOWN;

        // Jewel detection
        openCVCam.start(jewelDetector);
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 5000)
        {
            jewelOrder = jewelDetector.getCurrentOrder();

            if (jewelOrder != JewelDetector.JewelOrder.UNKNOWN)
                break;

            flow.yield();
        }
        openCVCam.stop();

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
            double[] DEPOSIT_LOCATIONS = {65.2, 81.2, 89.8};

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
//            robot.flipper.setGlyphHolderUpTo(true);

            // Align wheels backward.
            robot.swomniDrive.orientSwerveModules(Vector2D.polar(1, 180), 10, 1500, flow);

            // Drive back to the cryptobox.
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

            // Turn for better glyph placement
            double desiredHeading = getAlliance() == Alliance.BLUE ? 330 : 30;
            robot.swomniDrive.setDesiredHeading(desiredHeading);
            while (Math.abs(robot.gyro.getHeading() - desiredHeading) > 3)
                robot.swomniDrive.synchronousUpdate();
            robot.swomniDrive.stop();

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
        }

        // TODO Pain in the A** autonomous
        else if (getBalancePlate() == BalancePlate.TOP)
        {}

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
