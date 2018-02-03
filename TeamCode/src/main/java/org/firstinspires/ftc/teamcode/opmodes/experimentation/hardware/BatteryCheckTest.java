package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;

import hankextensions.EnhancedOpMode;

@Autonomous(name="Battery Checker", group=Constants.FINAL_BOT_EXPERIMENTATION)
public class BatteryCheckTest extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        ProcessConsole batteryConsole = log.newProcessConsole("Battery");
        while (true)
        {
            batteryConsole.write("It's " + FtcEventLoopHandler.latestBatterySend);

            flow.yield();
        }
    }
}
