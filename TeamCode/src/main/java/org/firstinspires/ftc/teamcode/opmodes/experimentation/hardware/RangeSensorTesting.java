package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;

import hankextensions.EnhancedOpMode;
import hankextensions.hardware.SmarterRangeSensor;

@Autonomous(name="Range Sensor Testing", group= Constants.EXPERIMENTATION)
public class RangeSensorTesting extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        SmarterRangeSensor frontRangeSensor = new SmarterRangeSensor(hardware.initialize(ModernRoboticsI2cRangeSensor.class, "Front Range Sensor"), 0x10);

        ProcessConsole rangeSensorConsole = log.newProcessConsole("Range Console");

        while (true)
        {
            rangeSensorConsole.write("Status of front: " + frontRangeSensor.rangeSensor.status());
            flow.yield();
        }
    }
}
