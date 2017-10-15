package ftc.vision;

import android.view.View;
import android.widget.FrameLayout;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


public class VuforiaManager
{
    private final FtcRobotControllerActivity activity;

    private VuforiaLocalizer vuforia = null;
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

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "your licence key";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        // Can also use a teapot.

        ImprovedVuforiaLocalizer vuforia = new ImprovedVuforiaLocalizer(parameters);

        vuforia.makeLoadingIndicator();
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
            //vuforia.close();
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
