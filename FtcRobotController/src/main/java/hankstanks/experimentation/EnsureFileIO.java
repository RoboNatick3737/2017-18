package hankstanks.experimentation;

import android.os.Environment;

import java.io.File;

import hankstanks.sdkextensions.Core;
import hankstanks.sdkextensions.files.FileManager;
import hankstanks.sdkextensions.threading.Flow;

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
