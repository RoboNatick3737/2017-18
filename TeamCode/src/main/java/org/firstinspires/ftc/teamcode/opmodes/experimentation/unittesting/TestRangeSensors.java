package org.firstinspires.ftc.teamcode.opmodes.experimentation.unittesting;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankextensions.EnhancedOpMode;

@Autonomous(name="Test Range Sensors", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class TestRangeSensors extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware, Robot.InitializationMode.AUTONOMOUS);

        ProcessConsole rangeConsole = log.newProcessConsole("Range Console");

        while (true)
        {
            rangeConsole.write(
                    "Front: " + (robot.frontRangeSensor != null ? robot.frontRangeSensor.getForwardDist() : 255),
                    "Back: " + (robot.backRangeSensor != null ? robot.backRangeSensor.getForwardDist() : 255));

            flow.yield();
        }
    }
}