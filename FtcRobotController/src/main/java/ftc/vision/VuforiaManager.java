package ftc.vision;

import android.view.View;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import com.qualcomm.ftcrobotcontroller.R;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VuforiaManager
{
    private final FtcRobotControllerActivity activity;

    private CloseableVuforiaLocalizer vuforia = null;
    private VuforiaTrackables trackables = null;

    public VuforiaManager(FtcRobotControllerActivity activity)
    {
        this.activity = activity;
    }

    public void enableAndShow()
    {
        if (vuforia != null)
            return;

        setViewStatus(true);

        // Create VuforiaLocalizer params.
        int cameraMonitorViewId = activity.getResources().getIdentifier("cameraMonitorViewId", "id", activity.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "Aey5dfP/////AAAAGZhLoGLnA0Rlqa+/KfzbrSVHIhN1VHeWSYvgUukYM+W7eoOKoUgT/Jwue7GkmHtn2bKRb9ETmlf2bQvkD5e7KpqUg1IT5Xdk6VE8CaXbcp+xjig6gBeH2Ydd8fYU3bZ5T1dul9+UAlJVw3n8X8232ljkiOsX8JwAgWvUY4W12rsfzpluCLhmKb1haJm/e4q2qMt+PwrhJbGz7u0z+pQpNQBDRVm37K3o7NKk6Cclb8xTwvVlDX1CAJACp/s2/S3NKIcuWToZdnq7v4hRMlXQbJXkFS5oT3NyJSFk4KiTb2h0vUU1uGydzzukA8lWMdTnHPZ9CDK+NY7gr/DnY5/0UwFi1QbaSzvug5W/l9wsrTIn";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        vuforia = new CloseableVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1); // 1 vision target.

        trackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        trackables.setName("Vision Targets");
        trackables.activate();
    }

    public VuforiaTrackables getTrackables() {
        if (vuforia != null) {
            return trackables;
        }

        return null;
    }

    public void disableAndHide()
    {
        if (vuforia != null)
        {
            vuforia.close();
            vuforia = null;
        }

        setViewStatus(false);
    }

    private FrameLayout layout;
    public void setViewStatus(boolean state)
    {
        if (layout == null)
            layout = (FrameLayout) activity.findViewById(R.id.vuforiaCamParent);

        int desiredView = state ? View.VISIBLE : View.INVISIBLE;

        layout.setVisibility(desiredView);
        layout.setEnabled(state);

        for (int i = 0; i < layout.getChildCount(); i++) {
            View child = layout.getChildAt(i);
            child.setVisibility(desiredView);
            child.setEnabled(state);
        }
    }
}
