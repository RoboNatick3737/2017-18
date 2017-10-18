package ftc.vision;

import android.view.View;
import android.view.WindowManager;
import android.widget.FrameLayout;
import android.widget.TextView;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

/**
 * Singleton class instead of a static class because the BaseLoaderCallback doesn't like
 * when there is a static
 */
public class OpenCVCam
{
    public static OpenCVCam instance = null;

    private BaseLoaderCallback mLoaderCallback;

    // States of the code progression.
    private boolean currentlyActive = false;
    private boolean loadedOpenCV = false;

    // Picture dimensions for analysis
    private final int FRAME_WIDTH_REQUEST = 176;
    private final int FRAME_HEIGHT_REQUEST = 144;

    // Tag for logging
    private final String LOG_TAG = "OpenCV";

    private CameraBridgeViewBase cameraBridgeViewBase = null;
    public FrameGrabber frameGrabber = null;

    // The activity's current state.
    public enum State {
        CREATE,
        RESUME,
        PAUSE,
        DESTROY,
        WINDOW_FOCUS_CHANGE
    }
    private State currentState;

    public OpenCVCam()
    {
        instance = this;

        mLoaderCallback = new BaseLoaderCallback(FtcRobotControllerActivity.instance) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                        RobotLog.vv(LOG_TAG, "OpenCV Manager Connected");
                        //from now onwards, you can use OpenCV API
                        // Mat m = new Mat(5, 10, CvType.CV_8UC1, new Scalar(0));
                        loadedOpenCV = true;
                        if (currentlyActive)
                            setCameraViewState(true);
                        break;
                    case LoaderCallbackInterface.INIT_FAILED:
                        RobotLog.vv(LOG_TAG, "Init Failed");
                        break;
                    case LoaderCallbackInterface.INSTALL_CANCELED:
                        RobotLog.vv(LOG_TAG, "Install Cancelled");
                        break;
                    case LoaderCallbackInterface.INCOMPATIBLE_MANAGER_VERSION:
                        RobotLog.vv(LOG_TAG, "Incompatible Version");
                        break;
                    case LoaderCallbackInterface.MARKET_ERROR:
                        RobotLog.vv(LOG_TAG, "Market Error");
                        break;
                    default:
                        RobotLog.vv(LOG_TAG, "OpenCV Manager Install");
                        super.onManagerConnected(status);
                        break;
                }
            }
        };
    }

    /**
     * Starts OpenCV: ensures that the camera shows up on the Robot Controller app.
     */
    public void start()
    {
        if (currentlyActive)
            return;

        setViewStatus(true);
/*
        onCreate();

        // Set new activity state depending on the current state.
        switch (currentState)
        {
            case PAUSE:
                onPause();
                break;
            case RESUME:
                onResume();
                break;
            case CREATE:
                onCreate();
                break;
            case DESTROY:
                onDestroy();
                break;
        }
*/
        currentlyActive = true;
    }

    /**
     * Stops OpenCV and hides it from the Robot Controller.
     */
    public void stop()
    {
        if (!currentlyActive)
            return;

        onDestroy();

        setViewStatus(false);

        instance = null;

        currentlyActive = false;
    }

    private void setViewStatus(final boolean state)
    {
        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable()
        {
            @Override
            public void run()
            {
                FrameLayout layout = (FrameLayout) FtcRobotControllerActivity.instance.findViewById(R.id.openCVCam);

                int desiredView = state ? View.VISIBLE : View.INVISIBLE;

                layout.setVisibility(desiredView);
                layout.setEnabled(state);

                for (int i = 0; i < layout.getChildCount(); i++) {
                    View child = layout.getChildAt(i);
                    child.setVisibility(desiredView);
                    child.setEnabled(state);
                }
            }
        });
    }

    private void setCameraViewState(final boolean state)
    {
        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable()
        {
            @Override
            public void run() {
                if (state)
                    cameraBridgeViewBase.enableView();
                else
                    cameraBridgeViewBase.disableView();
            }
        });
    }

    private void onCreate()
    {
        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                FtcRobotControllerActivity.instance.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

                cameraBridgeViewBase = (JavaCameraView) FtcRobotControllerActivity.instance.findViewById(R.id.show_camera_activity_java_surface_view);
                frameGrabber = new FrameGrabber(cameraBridgeViewBase, FRAME_WIDTH_REQUEST, FRAME_HEIGHT_REQUEST);
                frameGrabber.setImageProcessor(new BeaconProcessor());

                // Determines whether the app saves every image it gets.
                frameGrabber.setSaveImages(false);
            }
        });

    }

    private void onResume()
    {
        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (loadedOpenCV && !currentlyActive)
                    setCameraViewState(true);

                if (!OpenCVLoader.initDebug()) {
                    RobotLog.vv(LOG_TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
                    OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, FtcRobotControllerActivity.instance, mLoaderCallback);
                } else {
                    RobotLog.vv(LOG_TAG, "OpenCV library found inside package. Using it!");
                    mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
                }
            }
        });
    }

    public void onWindowFocusChanged(boolean hasFocus)
    {
        currentState = State.WINDOW_FOCUS_CHANGE;

        if (!currentlyActive)
            return;

//        if (hasFocus) {
//            frameGrabber.stopFrameGrabber();
//        } else {
//            frameGrabber.throwAwayFrames();
//        }
    }

    private void onPause()
    {
        currentState = State.PAUSE;

        if (cameraBridgeViewBase != null) {
            setCameraViewState(false);
        }
    }

    private void onDestroy()
    {
        currentState = State.DESTROY;

        onPause();

        setViewStatus(false);

        instance = null;
    }

    public void newActivityState(State state)
    {
        State initialState = currentState;
        currentState = state;

        switch (state)
        {
            case PAUSE:
                if (currentlyActive && initialState != State.PAUSE)
                    onPause();
                break;
            case RESUME:
                if (currentlyActive && initialState != State.RESUME)
                    onResume();
                break;
            case CREATE:
                if (currentlyActive && initialState == State.DESTROY)
                    onCreate();
                break;
            case DESTROY:
                if (currentlyActive)
                    onDestroy();
                break;
        }
    }

    //when the "Grab" button is pressed
    private boolean currentlyProcessingFrame = false;
    public void frameButtonOnClick(View v){
        if (currentlyProcessingFrame) {
            return;
        }

        currentlyProcessingFrame = true;

        frameGrabber.grabSingleFrame();
        while (!frameGrabber.isResultReady()) {
            try {
                Thread.sleep(5); //sleep for 5 milliseconds
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        Object result = frameGrabber.getResult();
        ((TextView) FtcRobotControllerActivity.instance.findViewById(R.id.resultText)).setText(result.toString());

        currentlyProcessingFrame = false;
    }
}
