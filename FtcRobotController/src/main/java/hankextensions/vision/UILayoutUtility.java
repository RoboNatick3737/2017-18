package hankextensions.vision;

import android.view.View;
import android.view.ViewGroup;

import com.qualcomm.ftcrobotcontroller.R;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public class UILayoutUtility
{
    /**
     * Enables/disables the FrameLayout instance and its children on the XML layout of the robot
     * controller.
     * @param state true = enabled, false = disabled.
     */
    public static void setFTCStuffVisibilityTo(final boolean state)
    {
        if (!state)
        {
            setLayoutVisibilityTo(R.id.boringFTCStuff, false, true);
        }
        else
        {
            setLayoutVisibilityTo(R.id.boringFTCStuff, true, true);
        }
    }

    /**
     * Disables/enables a given layout and its children.
     */
    public static void setLayoutVisibilityTo(int layoutID, boolean state, boolean includeChildren)
    {
        ViewGroup layout = (ViewGroup) FtcRobotControllerActivity.instance.findViewById(layoutID);

        int desiredView = state ? View.VISIBLE : View.GONE;

        layout.setVisibility(desiredView);
        layout.setEnabled(state);

        if (includeChildren)
        {
            for (int i = 0; i < layout.getChildCount(); i++)
            {
                View child = layout.getChildAt(i);
                child.setVisibility(desiredView);
                child.setEnabled(state);
            }
        }
    }
}
