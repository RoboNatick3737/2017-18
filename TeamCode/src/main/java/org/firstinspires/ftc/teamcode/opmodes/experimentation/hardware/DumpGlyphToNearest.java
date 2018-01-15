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

        double forwardSpeed = 1, horizontalSpeed = 1;

        while (Math.abs(forwardSpeed) > 0.01 || Math.abs(horizontalSpeed) > 0.01)
        {
            forwardSpeed = 0;
            if (tracker.estimatedForwardDistance > .32)
                forwardSpeed = -.1;

            horizontalSpeed = -0.7 * tracker.closestPlacementLocationOffset;

            robot.swerveDrive.setDesiredMovement(Vector2D.rectangular(forwardSpeed, horizontalSpeed));

            robot.swerveDrive.synchronousUpdate();

            flow.yield();
        }

        robot.swerveDrive.stop();

        robot.intake.intake();

        flow.msPause(1000);

        robot.flipper.advanceStage(2);

        flow.msPause(1000);
    }
}
