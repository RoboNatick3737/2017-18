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

public class OpenCVManager
{
    private final FtcRobotControllerActivity activity;
    private BaseLoaderCallback mLoaderCallback;

    // Picture dimensions for analysis
    private static final int FRAME_WIDTH_REQUEST = 176;
    private static final int FRAME_HEIGHT_REQUEST = 144;

    // Tag for logging
    private static final String LOG_TAG = "OpenCV";

    private CameraBridgeViewBase cameraBridgeViewBase = null;
    public FrameGrabber frameGrabber = null;

    public OpenCVManager(FtcRobotControllerActivity activity)
    {
        this.activity = activity;

        mLoaderCallback = new BaseLoaderCallback(activity) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                        RobotLog.vv(LOG_TAG, "OpenCV Manager Connected");
                        //from now onwards, you can use OpenCV API
                        // Mat m = new Mat(5, 10, CvType.CV_8UC1, new Scalar(0));
                        cameraBridgeViewBase.enableView();
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
     * Disables the OpenCV viewer from the app screen, and stops OpenCV.
     */
    public void disableAndHide()
    {
        onDestroy();

        setViewStatus(false);
    }

    /**
     * Enables the OpenCV viewer on the app screen, and starts OpenCV.
     */
    public void enableAndShow()
    {
        setViewStatus(true);

        onCreate();
    }

    private FrameLayout layout;
    public void setViewStatus(boolean state)
    {
        if (layout == null)
            layout = (FrameLayout) activity.findViewById(R.id.openCVCam);

        int desiredView = state ? View.VISIBLE : View.INVISIBLE;

        layout.setVisibility(desiredView);
        layout.setEnabled(state);

        for (int i = 0; i < layout.getChildCount(); i++) {
            View child = layout.getChildAt(i);
            child.setVisibility(desiredView);
            child.setEnabled(state);
        }
    }

    public void onCreate()
    {
        activity.getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        cameraBridgeViewBase = (JavaCameraView) activity.findViewById(R.id.show_camera_activity_java_surface_view);
        frameGrabber = new FrameGrabber(cameraBridgeViewBase, FRAME_WIDTH_REQUEST, FRAME_HEIGHT_REQUEST);
        frameGrabber.setImageProcessor(new BeaconProcessor());

        // Determines whether the app saves every image it gets.
        frameGrabber.setSaveImages(false);
    }

    public void onResume()
    {
        if (!OpenCVLoader.initDebug()) {
            RobotLog.vv(LOG_TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, activity, mLoaderCallback);
        } else {
            RobotLog.vv(LOG_TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public void onWindowFocusChanged(boolean hasFocus){
        if (hasFocus) {
            frameGrabber.stopFrameGrabber();
        } else {
            frameGrabber.throwAwayFrames();
        }
    }

    public void onPause(){
        if (cameraBridgeViewBase != null) {
            cameraBridgeViewBase.disableView();
        }
    }

    public void onDestroy()
    {
        if (cameraBridgeViewBase != null) {
            cameraBridgeViewBase.disableView();
            cameraBridgeViewBase = null;
        }
    }

    private boolean currentlyProcessingFrame = false;

    //when the "Grab" button is pressed
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
        ((TextView) activity.findViewById(R.id.resultText)).setText(result.toString());

        currentlyProcessingFrame = false;
    }
}
