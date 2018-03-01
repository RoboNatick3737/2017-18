package hankutanku.vision.opencv;

import android.content.Context;
import android.hardware.Camera;
import android.util.AttributeSet;

import org.opencv.android.JavaCameraView;

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

    public void setCameraResolution(int width, int height)
    {
        boolean foundSize = false;
        for (Camera.Size size : getSupportedPreviewSizes())
        {
            if (size.width == width && size.height == height)
            {
                foundSize = true;
                break;
            }
        }

        if (foundSize)
        {
//            mCamera.getParameters().setPreviewSize(width, height);
//            setMaxFrameSize(width, height);
//            disableView();
//            enableView();
        }
    }
}
