package org.firstinspires.ftc.teamcode.experimentation;

import android.os.Environment;

import java.io.File;

import hankstanks.hankextensions.Core;
import hankstanks.hankextensions.files.FileManager;
import hankstanks.hankextensions.threading.Flow;

public class EnsureFileIO extends Core
{
    protected void START() throws InterruptedException
    {
        try
        {
            log.lines("Beginning copy...");
            FileManager.CopyRAWtoSDCard(com.qualcomm.ftcrobotcontroller.R.raw.ussranthem, "FIRST/ussranthem.mp3");
            log.lines("Checking for " + Environment.getExternalStorageDirectory() + "/FIRST/ussranthem.mp3");
            if (new File(Environment.getExternalStorageDirectory() + "/FIRST/ussranthem.mp3").exists()) {
                log.lines("Success!");
            } else {
                log.lines("Not found.");
            }
        }
        catch (Exception e)
        {
            log.lines("Failed :( exception");
        }

        while (true)
            Flow.msPause(2000);
    }
}
