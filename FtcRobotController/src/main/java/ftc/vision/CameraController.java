package ftc.vision;

import android.view.View;
import android.widget.Button;

import com.qualcomm.ftcrobotcontroller.R;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public class CameraController
{
    private FtcRobotControllerActivity activity;
    private Button switchButton;

    // Both classes control their respective camera software.
    public final OpenCVManager openCVManager;
    public final VuforiaManager vuforiaManager;

    public enum Status {
        CREATE,
        RESUME,
        PAUSE,
        DESTROY
    }
    private Status currentStatus;

    // The current viewer being used (OpenCV/Vuforia)
    public enum Viewer {
        OPEN_CV,
        VUFORIA,
        PENDING
    }
    private Viewer currentViewer;


    public CameraController(FtcRobotControllerActivity activity, Viewer initialViewer)
    {
        this.activity = activity;

        // Create the two camera software managers.
        openCVManager = new OpenCVManager(activity);
        vuforiaManager = new VuforiaManager(activity);

        // Find the button (so that we can change the text in accordance with the current software).
        switchButton = ((Button)activity.findViewById(R.id.cameraSwitchButton));

        // Switch to the correct viewer.
        vuforiaManager.disableAndHide();
        openCVManager.disableAndHide();
        currentViewer = initialViewer;
        updateButtonText();
    }

    private void updateButtonText()
    {
        if (currentViewer == Viewer.OPEN_CV)
        {
            switchButton.setText("OpenCV");
        }
        else if (currentViewer == Viewer.VUFORIA)
        {
            switchButton.setText("Vuforia");
        }
        else if (currentViewer == Viewer.PENDING)
        {
            switchButton.setText("Pending...");
        }
    }

    public Viewer getViewer() {
        return currentViewer;
    }

    public void toggleViewer()
    {
        if (getViewer() == Viewer.OPEN_CV)
            initVuforia();
        else if (getViewer() == Viewer.VUFORIA)
            initOpenCV();
    }

    public void initVuforia()
    {
        // Set Pending status.
        currentViewer = Viewer.PENDING;
        updateButtonText();

        openCVManager.disableAndHide();
//        vuforiaManager.enableAndShow();
        vuforiaManager.setViewStatus(true);

        // Set Vuforia status.
        currentViewer = Viewer.VUFORIA;
        updateButtonText();
    }

    public void initOpenCV()
    {
        // Set Pending status.
        currentViewer = Viewer.PENDING;
        updateButtonText();

        vuforiaManager.disableAndHide();
        openCVManager.enableAndShow();
        if (currentStatus == Status.RESUME)
            openCVManager.onResume();

        // Set Vuforia status.
        currentViewer = Viewer.OPEN_CV;
        updateButtonText();
    }


    public void cameraOnCreate()
    {
        currentStatus = Status.CREATE;

        if (currentViewer == Viewer.OPEN_CV)
            initOpenCV();
    }

    public void cameraOnResume()
    {
        currentStatus = Status.RESUME;

        if (currentViewer == Viewer.OPEN_CV)
            openCVManager.onResume();

        else if (currentViewer == Viewer.VUFORIA)
            initVuforia();
    }

    public void cameraOnPause()
    {
        currentStatus = Status.PAUSE;

        if (currentViewer == Viewer.OPEN_CV)
            openCVManager.onPause();
    }

    public void cameraOnDestroy()
    {
        currentStatus = Status.DESTROY;

        if (currentViewer == Viewer.OPEN_CV)
            openCVManager.disableAndHide();

        else if (currentViewer == Viewer.VUFORIA)
            vuforiaManager.disableAndHide();
    }

    public void cameraOnWindowFocusChanged(boolean hasFocus)
    {
        if (currentViewer == Viewer.OPEN_CV)
            openCVManager.onWindowFocusChanged(hasFocus);
    }

    public void grabNewFrame(View v)
    {
        if (currentViewer == Viewer.OPEN_CV)
            openCVManager.frameButtonOnClick(v);
    }
}
