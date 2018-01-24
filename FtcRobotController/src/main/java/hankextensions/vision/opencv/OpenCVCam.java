package hankextensions.vision.opencv;

import android.view.View;
import android.view.WindowManager;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.UILayoutUtility;

/**
 * Singleton class instead of a static class because the BaseLoaderCallback doesn't like
 * when there is a static
 */
public class OpenCVCam implements VisionOpMode
{
    // Singleton class.
    public static OpenCVCam instance = null;

    // Used for the initialization of the CV libs.
    private BaseLoaderCallback mLoaderCallback;

    // States of the code progression.
    private boolean currentlyActive = false, loadingComplete = false;

    // Front vs. back camera
    public enum CameraPosition {FRONT, BACK}

    // Tag for file logging
    private final String LOG_TAG = "OpenCVCam";

    // The camera view.
    private CameraBridgeViewBase cameraBridgeViewBase = null;

    // The activity's current state.
    public enum State {
        CREATE,
        RESUME,
        PAUSE,
        DESTROY
    }
    private State currentState;

    /**
     * Registers the CV initialization callback, defines the singleton, and calls onCreate(), etc.
     */
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
                        EnhancedOpMode.instance.log.lines("OpenCV loaded from internal package successfully!");
                        loadingComplete = true;
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

        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                onCreate();
                onResume();
            }
        });
    }

    /**
     * Starts the camera view feed and streams input to a listener (defined elsewhere).
     * @param listener  The CvCameraViewListener to which input will be streamed.
     */
    public void start(VisionOpMode listener) throws InterruptedException
    {
        if (currentlyActive)
        {
            LoggingBase.instance.lines("Can't start with listener " + listener.getClass().toString() + " because already running");
            return;
        }
        currentlyActive = true;

        LoggingBase.instance.lines("Starting with listener " + listener.getClass().toString());

        // Enable the view and start the camera.  This NEEDS to move procedurally, so we run them synchronously on the UI thread.
        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                setViewStatus(true);
            }
        });

        // Wait for loading to complete.
        while (!loadingComplete)
            EnhancedOpMode.instance.flow.yield();

        if (cameraBridgeViewBase != null)
        {
            int maxWidth = 800, maxHeight = 600;
            if (listener.idealViewResolution() != null)
            {
                maxWidth = (int)(listener.idealViewResolution().width);
                maxHeight = (int)(listener.idealViewResolution().height);
            }
            cameraBridgeViewBase.setMaxFrameSize(maxWidth, maxHeight);
            cameraBridgeViewBase.setCvCameraViewListener(listener);
            cameraBridgeViewBase.setCameraIndex(listener.viewLocation() == CameraPosition.FRONT ? 1 : 0);
            setCameraViewState(true); // Might have to run on main activity.
        }
        else
            LoggingBase.instance.lines("Camera Bridge View Base was null, couldn't enable listener!");
    }

    /**
     * Stops OpenCV and hides it from the Robot Controller.
     */
    public void stop()
    {
        if (!currentlyActive)
            return;

        currentlyActive = false;

        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                setCameraViewState(false);
                setViewStatus(false);
            }
        });
    }

    /**
     * Enables/disables the FrameLayout instance and its children on the XML layout of the robot
     * controller.
     * @param state true = enabled, false = disabled.
     */
    private void setViewStatus(final boolean state)
    {
        // Enable at the start if we're turning this on.
        if (state)
        {
            cameraBridgeViewBase.setVisibility(View.VISIBLE);
        }
        else
        {
            cameraBridgeViewBase.setVisibility(View.GONE);
        }
    }

    /**
     * Enables/disables the camera view itself on the RC screen.
     * @param state true = enabled, false = disabled.
     */
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

    /**
     * Called when the Activity instance changes states from pause/resume/stop etc.
     * @param state the new state which has been switched to.
     */
    public void newActivityState(final State state)
    {
        LoggingBase.instance.lines("Activity state change to " + state.toString());

        FtcRobotControllerActivity.instance.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                switch (state) {
//                    case PAUSE:
//                        if (currentlyActive && currentState != State.PAUSE)
//                            onPause();
//                        break;
//                    case RESUME:
//                        if (currentlyActive && currentState != State.RESUME)
//                            onResume();
//                        break;
                    case DESTROY:
                        stop();
                        instance = null;
                        break;
                }
            }
        });

        currentState = state;
    }

    /**
     * Initializes the view, and gets the CameraBridgeViewBase and JavaCameraView which constitute
     * the OpenCV parts of the app.
     */
    private void onCreate()
    {
        LoggingBase.instance.lines("onCreate()");

        FtcRobotControllerActivity.instance.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        cameraBridgeViewBase = (CameraBridgeViewBase) FtcRobotControllerActivity.instance.findViewById(R.id.opencvJavaCameraView);
        cameraBridgeViewBase.setMaxFrameSize(800, 600); // default size
    }

    /**
     * Re-initializes the app (I think, idk for certain)
     */
    private void onResume()
    {
        LoggingBase.instance.lines("onResume()");
        currentState = State.RESUME;

        if (!OpenCVLoader.initDebug()) {
            RobotLog.vv(LOG_TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_4_0, FtcRobotControllerActivity.instance, mLoaderCallback);
        } else {
            RobotLog.vv(LOG_TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    /**
     * Called when the app is paused by the user.
     */
    private void onPause()
    {
        LoggingBase.instance.lines("onPause()");
        currentState = State.PAUSE;

        setCameraViewState(false);
    }

    // region OpenCV Methods (example listener)
    public void onCameraViewStarted(int width, int height) {}
    public void onCameraViewStopped() {}

    @Override
    public Size idealViewResolution()
    {
        return null;
    }

    @Override
    public CameraPosition viewLocation() {
        return null;
    }

    @Override
    public boolean enableCameraFlash() {
        return false;
    }

    /**
     * When the JavaCameraView sees a new frame (called very often).  This method has to
     * be modified in order to view single images.
     *
     * @param inputFrame the pixel array which the camera currently sees.
     * @return
     */
    public Mat onCameraFrame(Mat inputFrame)
    {
        return inputFrame;
    }
    // endregion
}
