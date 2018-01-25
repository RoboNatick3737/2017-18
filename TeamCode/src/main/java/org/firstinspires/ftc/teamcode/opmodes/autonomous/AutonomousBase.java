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

            // Choose the wheels which we'll use as an encoder anchor (observing their positions over time).
            SwerveModule[] wheels; // will be left wheels for red alliance, right for blue.
            if (getAlliance() == Alliance.RED)
            {
                wheels = new SwerveModule[2];
                wheels[0] = robot.swerveDrive.swerveModules[0]; // front left
                wheels[1] = robot.swerveDrive.swerveModules[1]; // back left
            }
            else
            {
                wheels = new SwerveModule[2];
                wheels[0] = robot.swerveDrive.swerveModules[2];
                wheels[1] = robot.swerveDrive.swerveModules[3];
            }

            // Get initial positions for wheels.
            double[] desiredPositions = new double[wheels.length];
            for (int i = 0; i < wheels.length; i++)
                desiredPositions[i] = wheels[i].driveMotor.motor.getCurrentPosition();

            // Choose the length to drive.
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
                    boolean allGood = true;
                    for (int i = 0; i < desiredPositions.length; i++)
                    {
                        if (wheels[i].driveMotor.motor.getCurrentPosition() > desiredPositions[i])
                        {
                            allGood = false;
                            break;
                        }
                    }

                    atAcceptableLocation = allGood;
                }
                // above desired positions
                else
                {
                    boolean allGood = true;
                    for (int i = 0; i < desiredPositions.length; i++)
                    {
                        if (wheels[i].driveMotor.motor.getCurrentPosition() < desiredPositions[i])
                        {
                            allGood = false;
                            break;
                        }
                    }

                    atAcceptableLocation = allGood;
                }

                robot.swerveDrive.synchronousUpdate();
                flow.yield();
            }

            // Dump glyph
            robot.flipper.advanceStage(2);
            flow.msPause(600);
        }

        // Pain in the A** autonomous
        else if (getBalancePlate() == BalancePlate.TOP)
        {
        }
        // endregion

        // region Multi-Glyph!
        // endregion
    }
}
