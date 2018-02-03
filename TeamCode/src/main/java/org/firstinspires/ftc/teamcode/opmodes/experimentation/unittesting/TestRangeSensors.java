package org.firstinspires.ftc.teamcode.opmodes.experimentation.unittesting;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import hankextensions.EnhancedOpMode;
import hankextensions.hardware.SmarterRangeSensor;

@Autonomous(name="Test Range Sensors", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class TestRangeSensors extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        // Get front and back sensors.
//        SmarterRangeSensor frontRangeSensor = new SmarterRangeSensor(hardware.initialize(ModernRoboticsI2cRangeSensor.class, "Front Range Sensor"), 0x10);
        SmarterRangeSensor backRangeSensor = new SmarterRangeSensor(hardware.initialize(ModernRoboticsI2cRangeSensor.class, "Back Range Sensor"), 0x2c);

        // For logging the perceived distances of the sensors.
        ProcessConsole rangeConsole = log.newProcessConsole("Range Console");

        while (true)
        {
            rangeConsole.write(
//                    "Front: " + frontRangeSensor.getForwardDist(),
                    "Back: " + backRangeSensor.getForwardDist());

            flow.yield();
        }
    }
}