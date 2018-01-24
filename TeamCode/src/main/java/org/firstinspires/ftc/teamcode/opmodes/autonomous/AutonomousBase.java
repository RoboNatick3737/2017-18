package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankextensions.EnhancedOpMode;
import hankextensions.structs.Vector2D;
import hankextensions.vision.opencv.OpenCVCam;

import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveModule;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.CVCryptoKeyDetector;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.JewelAndCryptoKeyTracker;
import org.firstinspires.ftc.teamcode.vision.relicrecoveryvisionpipelines.JewelDetector;

public abstract class AutonomousBase extends EnhancedOpMode implements CompetitionProgram
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
        Robot robot = new Robot(hardware, Robot.ControlMode.AUTONOMOUS);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        // region Initialization Detection of the Crypto Key and the Jewel Alignment
        JewelAndCryptoKeyTracker initializationObserver = new JewelAndCryptoKeyTracker();
        OpenCVCam cam = new OpenCVCam();
        cam.start(initializationObserver);

        CVCryptoKeyDetector.DetectedKey detectedKey = CVCryptoKeyDetector.DetectedKey.UNKNOWN;
        JewelDetector.JewelOrder jewelOrder = JewelDetector.JewelOrder.UNKNOWN;
        while (!isStarted()) // Runs until OpMode is started, then just goes from there.
        {
            JewelDetector.JewelOrder currentOrder = initializationObserver.jewelDetector.getCurrentOrder();
            if (currentOrder != JewelDetector.JewelOrder.UNKNOWN) // don't override valid value with unknown
                jewelOrder = currentOrder;

            CVCryptoKeyDetector.DetectedKey currentKey = initializationObserver.keyDetector.getLastDetected();
            if (currentKey != CVCryptoKeyDetector.DetectedKey.UNKNOWN)
                detectedKey = currentKey;

            flow.yield();
        }
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
        // Simple Autonomous
        if (getBalancePlate() == BalancePlate.BOTTOM)
        {
            robot.swerveDrive.setDesiredMovement(Vector2D.polar(0.5, getAlliance() == Alliance.RED ? 270 : 90));
            SwerveModule wheel1, wheel2; // will be left wheels for red alliance, right for blue.
            if (getAlliance() == Alliance.RED)
            {
                wheel1 = robot.swerveDrive.swerveModules[0]; // front left
                wheel2 = robot.swerveDrive.swerveModules[1]; // back left
            }
            else
            {
                wheel1 = robot.swerveDrive.swerveModules[2];
                wheel2 = robot.swerveDrive.swerveModules[3];
            }
            double[] desiredPositions = {wheel1.driveMotor.motor.getCurrentPosition(), wheel2.driveMotor.motor.getCurrentPosition()};
            double desiredDriveLength = 0;
            if (getAlliance() == Alliance.BLUE)
            {
                switch (detectedKey)
                {
                    case LEFT:
                        desiredDriveLength = 300;
                        break;

                    case CENTER:
                        desiredDriveLength = 400;
                        break;

                    case RIGHT:
                        desiredDriveLength = 500;
                        break;
                }
            }
            else
            {
                switch (detectedKey)
                {
                    case LEFT:
                        desiredDriveLength = -500;
                        break;

                    case CENTER:
                        desiredDriveLength = -400;
                        break;

                    case RIGHT:
                        desiredDriveLength = -300;
                        break;
                }
            }

            // Determine desired drive positions for both now.
            for (int i = 0; i < desiredPositions.length; i++)
                desiredPositions[i] += desiredDriveLength;

            // Drive to those positions.
            boolean atAcceptableLocation = false;
            while (!atAcceptableLocation)
            {
                // actual positions must be less than the desired positions.
                if (desiredDriveLength < 0)
                {
                }
                // above desired positions
                else
                {

                }

                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }
        }

        // Pain in the A** autonomous
        else if (getBalancePlate() == BalancePlate.TOP)
        {

        }
        // endregion
    }

    protected void legacy() throws InterruptedException
    {
        long start; // for timed stuff.

        // Initialize the robot.
        Robot robot = new Robot(hardware, Robot.ControlMode.AUTONOMOUS);

        // We're in auto, after all.
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        // Init the jewel detector (saves time)
        JewelDetector jewelDetector = new JewelDetector();
        OpenCVCam openCVCam = new OpenCVCam();
        openCVCam.start(jewelDetector);

        // Wait for the auto start period.
        waitForStart();

        // Tells the gyro that we haven't moved yet, so any difference in value it's experienced is incorrect.
        robot.gyro.startAntiDrift();

        // Get the jewel position.
        JewelDetector.JewelOrder currentOrder = JewelDetector.JewelOrder.UNKNOWN;
        start = System.currentTimeMillis();
        while (currentOrder == JewelDetector.JewelOrder.UNKNOWN && System.currentTimeMillis() - start < 5000)
        {
            currentOrder = jewelDetector.getCurrentOrder();
            flow.yield();
        }
        JewelDetector.JewelOrder determinedJewelOrder = currentOrder;
        openCVCam.stop();
        log.lines("Jewel order: " + determinedJewelOrder.toString());

        // Knock off the jewel as quickly as possible, but skip if we couldn't tell the ball orientation.
        if (determinedJewelOrder != JewelDetector.JewelOrder.UNKNOWN)
        {
            // Determine which direction we're going to have to rotate when auto starts.
            if (getAlliance() == Alliance.RED) // since this extends competition op mode.
            {
                if (determinedJewelOrder == JewelDetector.JewelOrder.BLUE_RED)
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.RIGHT, flow);
                else
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.LEFT, flow);

            }
            else if (getAlliance() == Alliance.BLUE)
            {
                if (determinedJewelOrder == JewelDetector.JewelOrder.BLUE_RED)
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.LEFT, flow);
                else
                    robot.ballKnocker.knockBall(BallKnocker.KnockerPosition.RIGHT, flow);
            }
        }

        // Drive off of the balance board.
        robot.swerveDrive.setDesiredMovement(Vector2D.polar(0.5, getAlliance() == Alliance.RED ? 270 : 90));
        start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < (getBalancePlate() == BalancePlate.BOTTOM ? 3000 : 1000))
        {
            robot.swerveDrive.synchronousUpdate();
            flow.yield();
        }

        // Rotate to face the cryptobox if we're right next to it.
        if (getBalancePlate() == BalancePlate.TOP)
        {
            double desiredHeading;
            if (getAlliance() == Alliance.RED)
                desiredHeading = 90;
            else
                desiredHeading = 270;

            robot.swerveDrive.setDesiredHeading(desiredHeading);

            while (Math.abs(robot.gyro.getHeading() - desiredHeading) > 5) {
                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }
        }

        robot.swerveDrive.stop();

        robot.intake.intake();

        flow.msPause(1000);

        robot.flipper.advanceStage(2);

        flow.msPause(1000);
    }
}
