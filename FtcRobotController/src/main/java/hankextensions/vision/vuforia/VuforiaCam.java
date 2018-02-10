package hankextensions.vision.vuforia;

import com.makiah.makiahsandroidlib.threading.Flow;
import com.qualcomm.ftcrobotcontroller.R;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import hankextensions.vision.UILayoutUtility;

/**
 * This class manages Vuforia, and everything associated with it for the UI.
 */
public class VuforiaCam
{
    public static VuforiaCam instance = null;

    private boolean currentlyActive = false;

    private CloseableVuforiaLocalizer vuforia = null;
    private VuforiaTrackables trackables = null;

    public VuforiaCam()
    {
        instance = this;
    }

    public void start()
    {
        start(false);
    }
    public void start(boolean useFrontCam)
    {
        currentlyActive = true;

        // Show the layouts for the vuforia camera.
        setViewStatus(true);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.vuforiaCameraLayout);

        parameters.vuforiaLicenseKey = "Aey5dfP/////AAAAGZhLoGLnA0Rlqa+/KfzbrSVHIhN1VHeWSYvgUukYM+W7eoOKoUgT/Jwue7GkmHtn2bKRb9ETmlf2bQvkD5e7KpqUg1IT5Xdk6VE8CaXbcp+xjig6gBeH2Ydd8fYU3bZ5T1dul9+UAlJVw3n8X8232ljkiOsX8JwAgWvUY4W12rsfzpluCLhmKb1haJm/e4q2qMt+PwrhJbGz7u0z+pQpNQBDRVm37K3o7NKk6Cclb8xTwvVlDX1CAJACp/s2/S3NKIcuWToZdnq7v4hRMlXQbJXkFS5oT3NyJSFk4KiTb2h0vUU1uGydzzukA8lWMdTnHPZ9CDK+NY7gr/DnY5/0UwFi1QbaSzvug5W/l9wsrTIn";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = useFrontCam ? VuforiaLocalizer.CameraDirection.FRONT : VuforiaLocalizer.CameraDirection.BACK;

        vuforia = new CloseableVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        trackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = trackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    public VuforiaTrackables getTrackables()
    {
        if (vuforia != null && currentlyActive) {
            return trackables;
        }

        return null;
    }

    public void stop(Flow flow) throws InterruptedException
    {
        if (currentlyActive)
            flow.msPause(500);

        currentlyActive = false;

        if (vuforia != null)
        {
            vuforia.close();
            vuforia = null;
        }

        setViewStatus(false);

        instance = null;
    }

    // Created for the CameraController to have easy access.
    public void setViewStatus(final boolean state)
    {
        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable() {
            @Override
            public void run()
            {
                // Enable at the start if we're turning this on.
                if (state)
                {
                    UILayoutUtility.setLayoutVisibilityTo(R.id.vuforiaCamParent, true, true);
                }
                else
                {
                    UILayoutUtility.setLayoutVisibilityTo(R.id.vuforiaCamParent, false, true);
                }
            }
        });
    }
}
