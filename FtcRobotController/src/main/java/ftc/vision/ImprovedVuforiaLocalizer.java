package ftc.vision;

import android.graphics.PointF;
import android.view.Gravity;
import android.view.View;
import android.widget.LinearLayout;

import com.vuforia.CameraDevice;
import com.vuforia.Renderer;
import com.vuforia.Vec2I;
import com.vuforia.VideoBackgroundConfig;
import com.vuforia.VideoMode;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class ImprovedVuforiaLocalizer extends VuforiaLocalizerImpl
{
    boolean closed = false;

    public ImprovedVuforiaLocalizer(Parameters parameters)
    {
        super(parameters);
    }

    @Override
    public void makeLoadingIndicator()
    {
        super.makeLoadingIndicator();
    }

    @Override
    public void close() {
        if (!closed) super.close();
        closed = true;
    }

    @Override
    protected void configureVideoBackground()
    {
        if (glSurface == null)
            return;

        VideoBackgroundConfig videoBackgroundConfig = new VideoBackgroundConfig();
        videoBackgroundConfig.setPosition(new Vec2I(250, 250));
        videoBackgroundConfig.setSize(new Vec2I(1000, 1000));

        Renderer.getInstance().setVideoBackgroundConfig(videoBackgroundConfig);
    }
}