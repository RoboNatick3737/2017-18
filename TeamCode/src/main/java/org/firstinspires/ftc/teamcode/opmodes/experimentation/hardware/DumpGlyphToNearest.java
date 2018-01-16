package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.vision.analysis.CryptoboxTracker;

import hankextensions.EnhancedOpMode;
import hankextensions.structs.Vector2D;
import hankextensions.vision.opencv.OpenCVCam;

@Autonomous(name="Dump Glyph to Nearest", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class DumpGlyphToNearest extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        CryptoboxTracker tracker = new CryptoboxTracker();
        tracker.setAlliance(CompetitionProgram.Alliance.BLUE);
        tracker.setTrackingMode(CryptoboxTracker.ColumnTrackingMode.SIMPLE);
        tracker.setLoggingEnabledTo(true);

        OpenCVCam cam = new OpenCVCam();
        cam.start(tracker, true);

        robot.lights.setLightsTo(true);

        waitForStart();

        double offFromForwardIdeal = 1, offFromHorizontalIdeal = 1;

        // Double equality
        while (Math.abs(offFromForwardIdeal) > 0.03 || Math.abs(offFromHorizontalIdeal) > 0.02)
        {
            double horizontalSpeed, forwardSpeed;
            if (!tracker.detectedNoColumns)
            {
                offFromForwardIdeal = -(tracker.estimatedForwardDistance - .32);
                forwardSpeed = 0.7 * offFromForwardIdeal;
                if (Math.abs(forwardSpeed) < .03)
                    forwardSpeed = 0;

                offFromHorizontalIdeal = -tracker.closestPlacementLocationOffset;
                horizontalSpeed = 0.7 * offFromHorizontalIdeal;
                if (Math.abs(horizontalSpeed) < .03)
                    horizontalSpeed = 0;
            }
            else
            {
                horizontalSpeed = 0;
                forwardSpeed = 0.1;
            }

            robot.swerveDrive.setDesiredMovement(Vector2D.rectangular(forwardSpeed, horizontalSpeed));

            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() - start < 100) // Don't make super fast adjustments.
            {
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
