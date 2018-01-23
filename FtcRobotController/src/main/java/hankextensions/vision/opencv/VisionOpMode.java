package hankextensions.vision.opencv;

import android.support.annotation.NonNull;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Size;

public interface VisionOpMode extends CameraBridgeViewBase.CvCameraViewListener
{
    /**
     * The resolution at which the camera will be rendering.  A higher resolution is more detailed
     * (duh) but has a lower FPS.
     * @return  The max size for the camera display, return null for no preference.
     */
    Size idealViewResolution();

    /**
     * @return  The desired camera view for this OpMode (crypto detector is front, glyph is back).
     */
    OpenCVCam.CameraPosition viewLocation();

    /**
     * @return  Whether or not to enable back camera flash for this OpMode (not implemented yet).
     */
    boolean enableCameraFlash();
}
