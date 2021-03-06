package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import dude.makiah.androidlib.logging.ProcessConsole;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

import hankutanku.EnhancedOpMode;

@Autonomous(name="Battery Checker", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
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
