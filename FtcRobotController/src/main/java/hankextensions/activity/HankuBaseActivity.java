package hankextensions.activity;

import android.app.Activity;
import android.os.Bundle;

import com.makiah.makiahsandroidlib.threading.TaskParent;

import hankextensions.phonesensors.AndroidGyro;
import hankextensions.vision.opencv.OpenCVCam;
import hankextensions.vision.vuforia.VuforiaCam;

/**
 * Neat little brainchild I had: why not have all vision code in the base class of FtcRobotControllerActivity
 * and just override it?
 */
public abstract class HankuBaseActivity extends Activity implements TaskParent
{
    public static HankuBaseActivity instance;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);

        instance = this;
        OpenCVCam.instance = null;
        VuforiaCam.instance = null;
        AndroidGyro.instance = null;

        // Enable when stable.
        Thread.setDefaultUncaughtExceptionHandler(new DefaultExceptionHandler(this, this));
    }

    @Override
    protected void onResume() {
        super.onResume();

        if (OpenCVCam.instance != null)
            OpenCVCam.instance.newActivityState(OpenCVCam.State.RESUME);
    }

    @Override
    protected void onPause() {
        super.onPause();

        if (OpenCVCam.instance != null)
            OpenCVCam.instance.newActivityState(OpenCVCam.State.PAUSE);
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);

//        if (OpenCVCam.instance != null)
//            OpenCVCam.instance.onWindowFocusChanged(hasFocus);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        if (OpenCVCam.instance != null)
            OpenCVCam.instance.newActivityState(OpenCVCam.State.DESTROY);
    }

    @Override
    public boolean isTaskActive()
    {
        return true;
    }
}
