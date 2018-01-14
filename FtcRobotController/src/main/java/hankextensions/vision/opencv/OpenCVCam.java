package hankextensions.vision.opencv;

import android.view.View;
import android.view.WindowManager;
import android.widget.FrameLayout;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import hankextensions.EnhancedOpMode;

/**
 * Singleton class instead of a static class because the BaseLoaderCallback doesn't like
 * when there is a static
 */
public class OpenCVCam implements CameraBridgeViewBase.CvCameraViewListener
{
    // Picture dimensions for analysis
    private static final int FRAME_WIDTH_REQUEST = 600, FRAME_HEIGHT_REQUEST = 400;

    // Singleton class.
    public static OpenCVCam instance = null;

    private BaseLoaderCallback mLoaderCallback;

    // States of the code progression.
    private boolean currentlyActive = false, loadingComplete = false;

    // Tag for file logging
    private final String LOG_TAG = "OpenCVCam";

    // The camera view.
    private CameraBridgeViewBase cameraBridgeViewBase = null;

    private Mat cameraViewMat;

    // The activity's current state.
    public enum State {
        CREATE,
        RESUME,
        PAUSE,
        DESTROY
    }
    private State currentState;

    // Prepares the callback for OpenCV initialization.
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

    public void start(CameraBridgeViewBase.CvCameraViewListener listener) throws InterruptedException
    {
        start(listener, false);
    }
    public void start(CameraBridgeViewBase.CvCameraViewListener listener, boolean useFrontCamera) throws InterruptedException
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

        if (cameraBridgeViewBase != null) {
            cameraBridgeViewBase.setCvCameraViewListener(listener);
            cameraBridgeViewBase.setCameraIndex(useFrontCamera ? 1 : 0);
            setCameraViewState(true); // Might have to run on main activity.
        }
        else
            LoggingBase.instance.lines("Camera Bridge View Base was null, couldn't enable listener!");
    }

    //Stops OpenCV and hides it from the Robot Controller.
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

    // HAS to run on UI thread or view thread error.
    private void onCreate()
    {
        LoggingBase.instance.lines("onCreate()");

        FtcRobotControllerActivity.instance.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        cameraBridgeViewBase = (JavaCameraView) FtcRobotControllerActivity.instance.findViewById(R.id.show_camera_activity_java_surface_view);
//        cameraBridgeViewBase.setMaxFrameSize(FRAME_WIDTH_REQUEST, FRAME_HEIGHT_REQUEST);
    }

    private void onResume()
    {
        LoggingBase.instance.lines("onResume()");
        currentState = State.RESUME;

        if (!OpenCVLoader.initDebug()) {
            RobotLog.vv(LOG_TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, FtcRobotControllerActivity.instance, mLoaderCallback);
        } else {
            RobotLog.vv(LOG_TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    private void onPause()
    {
        LoggingBase.instance.lines("onPause()");
        currentState = State.PAUSE;

        setCameraViewState(false);
    }

    public void onCameraViewStarted(int width, int height)
    {
        cameraViewMat = new Mat(height, width, CvType.CV_8UC4);
    }

    public void onCameraViewStopped() {
        cameraViewMat.release();
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
        //RobotCore.instance.log.lines("Normal frame called");
        return inputFrame;
    }
}
