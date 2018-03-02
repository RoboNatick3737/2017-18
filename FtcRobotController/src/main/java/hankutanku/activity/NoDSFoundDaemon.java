package hankutanku.activity;

import android.app.Activity;
import android.app.AlarmManager;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;

import dude.makiah.androidlib.threading.ParallelTask;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

/**
 * Searches for an existent link between the RC and the DS and if no connection is established,
 * we're probably dead in the water and need to restart the controller.
 */
public class NoDSFoundDaemon extends ParallelTask
{
    private static final long WAIT_FOR_CONNECTION_DELAY = 20000;

    private final Activity toRestart;
    private final long START;

    public NoDSFoundDaemon(Activity toRestart)
    {
        super(HankuBaseActivity.instance, "No DS Restart Daemon");
        this.toRestart = toRestart;
        this.START = System.currentTimeMillis();
        this.run();
    }

    private boolean stopPendingRestart = false;
    public void stopPendingRestart()
    {
        stopPendingRestart = true;
    }

    public void restartRCApp() {
        try {
            // set up the Intent to restart the app
            Intent intent = new Intent(toRestart, FtcRobotControllerActivity.class);
            intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_CLEAR_TASK | Intent.FLAG_ACTIVITY_NEW_TASK);
            PendingIntent pendingIntent = PendingIntent.getActivity(toRestart, 0, intent, PendingIntent.FLAG_ONE_SHOT);

            // schedule the restart in two seconds
            AlarmManager mgr = (AlarmManager) toRestart.getSystemService(Context.ALARM_SERVICE);
            mgr.set(AlarmManager.RTC, System.currentTimeMillis() + 2000, pendingIntent);
            // finish the activity and close the dead app
            toRestart.finish();
            System.exit(2);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void onDoTask() throws InterruptedException
    {
        while (System.currentTimeMillis() - START < WAIT_FOR_CONNECTION_DELAY && !stopPendingRestart)
            flow.yield();

        if (!stopPendingRestart)
            restartRCApp();
    }
}
