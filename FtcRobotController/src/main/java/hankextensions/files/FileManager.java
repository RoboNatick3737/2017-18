package hankextensions.files;

import android.os.Environment;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

import hankextensions.Core;

public class FileManager
{
    public static void CopyRAWtoSDCard(int id, String internalPath) throws IOException
    {
        String actualPath = Environment.getExternalStorageDirectory() + "/" + internalPath;

        if (new File(actualPath).exists())
        {
            Core.log.lines("Can't copy " + internalPath + " since it already exists.");
        }

        InputStream in = Core.instance.hardwareMap.appContext.getResources().openRawResource(id);
        FileOutputStream out = new FileOutputStream(Environment.getExternalStorageDirectory() + "/" + internalPath);
        byte[] buff = new byte[1024];
        int read = 0;
        try {
            while ((read = in.read(buff)) > 0) {
                out.write(buff, 0, read);
            }
        } finally {
            in.close();
            out.close();
        }
    }
}
