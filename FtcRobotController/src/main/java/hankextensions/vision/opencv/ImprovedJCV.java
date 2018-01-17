package hankextensions.vision.opencv;

import android.content.Context;
import android.hardware.Camera;
import android.util.AttributeSet;

import org.opencv.android.JavaCameraView;
import org.opencv.core.Size;

import java.util.List;

/**
 * Actually changes the camera resolution thanks to http://www.thecodecity.com/2016/08/focus-modes-opencv-javacameraview-android.html
 * (no thanks to OpenCV >:( ).
 */
public class ImprovedJCV extends JavaCameraView
{
    public ImprovedJCV(Context context, AttributeSet attrs)
    {
        super(context, attrs);
    }

    public List<Camera.Size> getSupportedPreviewSizes()
    {
        return mCamera.getParameters().getSupportedPreviewSizes();
    }
}
