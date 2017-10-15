package ftc.vision;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import com.qualcomm.ftcrobotcontroller.R;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class CameraController
{
    private FtcRobotControllerActivity activity;

    private View appView;
    private Button switchButton;

    // Vuforia stuff.
    private CloseableVuforiaLocalizer vuforia;
    private void ensureVuforiaClosed() {
        if (vuforia != null) {
            vuforia.close();
            vuforia = null;
        }
    }
    public VuforiaTrackables getTrackables() {
        if (vuforia != null) {
            VuforiaTrackables trackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            trackables.setName("Vision Targets");

            return trackables;
        }

        return null;
    }

    public CameraController(FtcRobotControllerActivity activity)
    {
        this.activity = activity;

        LayoutInflater inflater = (LayoutInflater) activity.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        appView = inflater.inflate(R.layout.activity_ftc_controller, null);

        switchButton = ((Button)appView.findViewById(R.id.cameraSwitchButton));
    }

    public enum Viewer {
        OPEN_CV,
        VUFORIA,
        PENDING
    }

    private Viewer currentViewer;
    public Viewer getViewer() {
        return currentViewer;
    }

    public void initVuforia()
    {
        // Indicate to the primary activity that this is underway but not to accept further view transitions.
        currentViewer = Viewer.PENDING;

        // Set the UI to indicate the pending mode for this item.
        switchButton.setText("Pending");

        // Paranoia
        ensureVuforiaClosed();

        // Close the OpenCV cam.
        activity.myOnDestroy();
        FrameLayout layout = (FrameLayout) appView.findViewById(R.id.openCVCam);
        for (int i = 0; i < layout.getChildCount(); i++) {
            View child = layout.getChildAt(i);
            child.setEnabled(false);
        }

        // Create VuforiaLocalizer params.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        vuforia = new CloseableVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1); // 1 vision target.


        // Update currentViewer
        switchButton.setText("Vuforia");
        currentViewer = Viewer.VUFORIA;
    }

    public void initOpenCV()
    {
        // Set to pending so that no other switches can be made in this time.
        currentViewer = Viewer.PENDING;

        // Set the UI to indicate the pending mode for this item.
        switchButton.setText("Pending");

        ensureVuforiaClosed();

        // Disable Vuforia from view.
        LinearLayout vLayout = (LinearLayout) appView.findViewById(R.id.cameraMonitorViewId);
        for (int i = 0; i < vLayout.getChildCount(); i++) {
            View child = vLayout.getChildAt(i);
            child.setEnabled(false);
        }

        // Enable OpenCV.
        FrameLayout layout = (FrameLayout) appView.findViewById(R.id.openCVCam);
        for (int i = 0; i < layout.getChildCount(); i++) {
            View child = layout.getChildAt(i);
            child.setEnabled(true);
        }
        activity.myOnCreate();
    }
}
