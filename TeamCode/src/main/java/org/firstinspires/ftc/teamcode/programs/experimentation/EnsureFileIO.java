package org.firstinspires.ftc.teamcode.programs.experimentation;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.File;

import org.firstinspires.ftc.teamcode.programs.Core;
import org.firstinspires.ftc.teamcode.sdkextensions.files.FileManager;
import org.firstinspires.ftc.teamcode.sdkextensions.logging.Log;
import org.firstinspires.ftc.teamcode.sdkextensions.threading.Flow;

@Autonomous(name="Ensure File IO", group="Experimentation")
public class EnsureFileIO extends Core
{
    protected void START() throws InterruptedException
    {
        try
        {
            log.newLine("Beginning copy...");
            FileManager.CopyRAWtoSDCard(com.qualcomm.ftcrobotcontroller.R.raw.ussranthem, "FIRST/ussranthem.mp3");
            log.newLine("Checking for " + Environment.getExternalStorageDirectory() + "/FIRST/ussranthem.mp3");
            if (new File(Environment.getExternalStorageDirectory() + "/FIRST/ussranthem.mp3").exists()) {
                log.newLine("Success!");
            } else {
                log.newLine("Not found.");
            }
        }
        catch (Exception e)
        {
            log.newLine("Failed :( exception");
        }

        while (true)
            Flow.msPause(2000);
    }
}
