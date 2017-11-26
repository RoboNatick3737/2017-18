package hankextensions.vision.opencv;

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
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import hankextensions.RobotCore;
import hankextensions.vision.old.BeaconProcessor;
import hankextensions.vision.old.FrameGrabber;

import static org.opencv.core.CvType.CV_8UC3;

/**
 * Singleton class instead of a static class because the BaseLoaderCallback doesn't like
 * when there is a static
 */
public class OpenCVCam implements CameraBridgeViewBase.CvCameraViewListener2
{
    // Whether or not this camera is just supplying RGBA frames to the driver station or passing back frames to a listener.
    public enum CameraMode {REQUEST, CONTINUOUS}
    private CameraMode currentCameraMode = CameraMode.CONTINUOUS;
    private ArrayList<OpenCVMatReceiver> frameCallbacks = new ArrayList<>(); // It's possible that multiple might call.

    public static OpenCVCam instance = null;

    private BaseLoaderCallback mLoaderCallback;

    // States of the code progression.
    private boolean currentlyActive = false;
    // The activity's current state.
    public enum State {
        CREATE,
        RESUME,
        PAUSE,
        DESTROY
    }

    // Picture dimensions for analysis
    private final int FRAME_WIDTH_REQUEST = 176, FRAME_HEIGHT_REQUEST = 144;

    // Tag for file logging
    private final String LOG_TAG = "OpenCVCam";

    // The camera view.
    private CameraBridgeViewBase cameraBridgeViewBase = null;

    private Mat cameraViewMat, mRgbaF, mRgbaT;


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
                        RobotCore.instance.log.lines("loader callback");
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
        RobotCore.instance.log.lines("Activity requested " + state.toString());

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
        RobotCore.instance.log.lines("onCreate()");

        FtcRobotControllerActivity.instance.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        cameraBridgeViewBase = (JavaCameraView) FtcRobotControllerActivity.instance.findViewById(R.id.show_camera_activity_java_surface_view);
        cameraBridgeViewBase.setMinimumHeight(FRAME_HEIGHT_REQUEST);
        cameraBridgeViewBase.setMinimumWidth(FRAME_WIDTH_REQUEST);

        cameraBridgeViewBase.setCvCameraViewListener(this);
    }

    private void onResume()
    {
        RobotCore.instance.log.lines("onResume()");
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
    }

    private void onPause()
    {
        RobotCore.instance.log.lines("onPause()");
        currentState = State.PAUSE;

        setCameraViewState(false);
    }

    public void onCameraViewStarted(int width, int height)
    {
        cameraViewMat = new Mat(height, width, CvType.CV_8UC4);
        mRgbaF = new Mat(height, width, CvType.CV_8UC4);
        mRgbaT = new Mat(height, width, CvType.CV_8UC4);
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
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame)
    {
        // Pass back the last frame if we're in the request mode and we don't want a new frame.
        if (currentCameraMode == CameraMode.REQUEST && frameCallbacks.size() == 0)
        {
            // In case not even a single picture has been taken.
            if (cameraViewMat == null)
                cameraViewMat = new Mat(FRAME_HEIGHT_REQUEST, FRAME_WIDTH_REQUEST, CV_8UC3, new Scalar(0, 0, 0));

            return cameraViewMat;
        }

        // TODO Auto-generated method stub
        cameraViewMat = inputFrame.rgba();

        // Rotate cameraViewMat 90 degrees
        //Core.transpose(cameraViewMat, mRgbaT);
        //Imgproc.resize(mRgbaT, mRgbaF, mRgbaF.size(), 0, 0, 0);
        //Core.flip(mRgbaF, cameraViewMat, 1);

        if (currentCameraMode == CameraMode.REQUEST && frameCallbacks.size() > 0)
        {
            // Provide all callbacks the frame they requested.
            for (OpenCVMatReceiver callback : frameCallbacks)
                callback.receiveMat(cameraViewMat);

            // Remove all callbacks.
            frameCallbacks.clear();
        }

        return cameraViewMat;
    }

    public void setCameraMode(CameraMode mode)
    {
        if (currentCameraMode == mode)
            return;

        currentCameraMode = mode;

        if (currentCameraMode == CameraMode.REQUEST)
            RobotCore.instance.log.lines("Changed camera mode to request");
        else if (currentCameraMode == CameraMode.CONTINUOUS)
            RobotCore.instance.log.lines("Changed camera mode to continuous");

        // Ensure that we clear the list if we changed the mode.
        if (currentCameraMode == CameraMode.REQUEST)
            frameCallbacks.clear();
    }

    public void requestFrame(OpenCVMatReceiver callback)
    {
        frameCallbacks.add(callback);
    }
}
