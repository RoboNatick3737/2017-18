package hankutanku.vision;

import android.view.View;
import android.view.ViewGroup;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public class UILayoutUtility
{
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
