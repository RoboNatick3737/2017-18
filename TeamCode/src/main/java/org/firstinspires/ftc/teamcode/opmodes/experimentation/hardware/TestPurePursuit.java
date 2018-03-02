package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.SwomniDrive;

import dude.makiah.androidlib.threading.ScheduledTaskPackage;
import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;
import hankutanku.math.Function;
import hankutanku.math.ParametrizedVector;

@Autonomous(name="Test Pure Pursuit", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestPurePursuit extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        // Init the bot.
        final Robot robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS);
        robot.swomniDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);
        robot.swomniDrive.setSwomniControlMode(SwomniDrive.SwomniControlMode.SWERVE_DRIVE);
        robot.swomniDrive.setVectorControlBasedOnHeading(false);

        while (!isStarted())
            flow.yield();

        // Drive left
        robot.swomniDrive.purePursuit(ParametrizedVector.polar(
                new Function() {
                    @Override
                    public double value(double input) {
                        return input * 45;
                    }
                },
                new Function() {
                    @Override
                    public double value(double input) {
                        return 90;
                    }
                }),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 3),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 1),
                .1,
                null, flow);

        // Drive right
        robot.swomniDrive.purePursuit(ParametrizedVector.polar(
                new Function() {
                    @Override
                    public double value(double input) {
                        return input * 45;
                    }
                },
                new Function() {
                    @Override
                    public double value(double input) {
                        return 270;
                    }
                }),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 3),
                new TimeMeasure(TimeMeasure.Units.SECONDS, 1),
                .1,
                null, flow);
    }
}
