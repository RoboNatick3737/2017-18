package hankextensions.vision;

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

import hankextensions.logging.Log;

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

    // Picture dimensions for analysis
    private final int FRAME_WIDTH_REQUEST = 176, FRAME_HEIGHT_REQUEST = 144;

    // Tag for file logging
    private final String LOG_TAG = "OpenCV";

    private CameraBridgeViewBase cameraBridgeViewBase = null;
    public FrameGrabber frameGrabber = null;

    // The activity's current state.
    public enum State {
        CREATE,
        RESUME,
        PAUSE,
        DESTROY
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
                        Log.instance.lines("loader callback");
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

    //Starts OpenCV and ensures that the camera shows up on the Robot Controller app.
    public void start()
    {
        if (currentlyActive)
            return;

        currentlyActive = true;
        currentState = State.RESUME;

        // Enable the view and start the camera.  This NEEDS to move procedurally, so we run them synchronously on the UI thread.
        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                setViewStatus(true);
                onCreate();
                onResume();
            }
        });
    }

    //Stops OpenCV and hides it from the Robot Controller.
    public void stop()
    {
        if (!currentlyActive)
            return;

        instance = null;
        currentlyActive = false;

        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                setCameraViewState(false);
                setViewStatus(false);
            }
        });
    }

    // Enables/disables the FrameLayout and its children.
    private void setViewStatus(final boolean state)
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

    // Enables/disables the camera view on the RC app.
    private void setCameraViewState(boolean state)
    {
        // If we don't have the camera bridge view base set up or it's already set to the state we want, don't do anything.
        if (cameraBridgeViewBase == null)
            return;

        // Set camera view state.
        if (state)
            cameraBridgeViewBase.enableView();
        else
            cameraBridgeViewBase.disableView();
    }

    // Called when the FtcRobotControllerActivity changes activity states.
    public void newActivityState(final State state)
    {
        Log.instance.lines("Activity requested " + state.toString());

        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                switch (state) {
                    case PAUSE:
                        if (currentlyActive && currentState != State.PAUSE)
                            onPause();
                        break;
                    case RESUME:
                        if (currentlyActive && currentState != State.RESUME)
                            onResume();
                        break;
                    case DESTROY:
                        stop();
                        break;
                }
            }
        });

        currentState = state;
    }

    // HAS to run on UI thread or view thread error.
    private void onCreate()
    {
        Log.instance.lines("onCreate()");

        FtcRobotControllerActivity.instance.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        cameraBridgeViewBase = (JavaCameraView) FtcRobotControllerActivity.instance.findViewById(R.id.show_camera_activity_java_surface_view);
        frameGrabber = new FrameGrabber(cameraBridgeViewBase, FRAME_WIDTH_REQUEST, FRAME_HEIGHT_REQUEST);
        frameGrabber.setImageProcessor(new BeaconProcessor());

        // Determines whether the app saves every image it gets.
        frameGrabber.setSaveImages(false);
    }

    private void onResume()
    {
        Log.instance.lines("onResume()");
        currentState = State.RESUME;

        if (!OpenCVLoader.initDebug()) {
            RobotLog.vv(LOG_TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, FtcRobotControllerActivity.instance, mLoaderCallback);
        } else {
            RobotLog.vv(LOG_TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public void onWindowFocusChanged(boolean hasFocus)
    {
        if (!currentlyActive)
            return;

        if (hasFocus) {
            frameGrabber.stopFrameGrabber();
        } else {
            frameGrabber.throwAwayFrames();
        }
    }

    private void onPause()
    {
        Log.instance.lines("onPause()");
        currentState = State.PAUSE;

        setCameraViewState(false);
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
