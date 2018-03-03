package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.SwomniDrive;

import dude.makiah.androidlib.logging.ProcessConsole;
import dude.makiah.androidlib.threading.ScheduledTaskPackage;
import hankutanku.EnhancedOpMode;
import hankutanku.math.Vector2D;

@TeleOp(name="Test Displacement Odometry", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestDisplacementOdometry extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        // Init the bot.
        final Robot robot = new Robot(hardware, AutoOrTeleop.AUTONOMOUS);
        robot.swomniDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);
        robot.swomniDrive.setSwomniControlMode(SwomniDrive.SwomniControlMode.SWERVE_DRIVE);
        robot.swomniDrive.setJoystickControlMethod(SwomniDrive.JoystickControlMethod.ROBOT_CENTRIC);

        ProcessConsole totalDisplacement = log.newProcessConsole("Total Displacement Vector");

        while (true)
        {
            totalDisplacement.write(
                    "Total: " + Vector2D.average(
                                robot.swomniDrive.swomniModules[0].getDisplacementVector(),
                                robot.swomniDrive.swomniModules[1].getDisplacementVector(),
                                robot.swomniDrive.swomniModules[2].getDisplacementVector(),
                                robot.swomniDrive.swomniModules[3].getDisplacementVector())
            );

            robot.swomniDrive.synchronousUpdate();
            flow.yield();
        }
    }
}
