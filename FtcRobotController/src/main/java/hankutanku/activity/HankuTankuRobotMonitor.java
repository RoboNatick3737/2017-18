package hankutanku.activity;

import android.app.Activity;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftccommon.external.SoundPlayingRobotMonitor;
import org.firstinspires.ftc.robotcore.internal.network.PeerStatus;

public class HankuTankuRobotMonitor extends SoundPlayingRobotMonitor
{
    public static boolean gotDisconnect = false;

    private NoDSFoundDaemon noDSFoundDaemon = null;
    private final Activity toRestart;

    public HankuTankuRobotMonitor(Activity toRestart)
    {
        super();
        this.toRestart = toRestart;
        this.noDSFoundDaemon = new NoDSFoundDaemon(this.toRestart);
    }

    @Override public void updatePeerStatus(@NonNull PeerStatus peerStatus)
    {
        if (peerStatus != this.peerStatus)
        {
            if (DEBUG) RobotLog.vv(SoundPlayer.TAG, "updatePeerStatus(%s)", peerStatus.toString());
            switch (peerStatus)
            {
                case UNKNOWN:
                    break;

                case CONNECTED:
                    if (noDSFoundDaemon != null)
                        noDSFoundDaemon.stopPendingRestart();
                    noDSFoundDaemon = null;

                    playSound(soundConnect);
                    break;

                case DISCONNECTED:
                    if (noDSFoundDaemon == null)
                        noDSFoundDaemon = new NoDSFoundDaemon(toRestart);

                    playSound(soundDisconnect);
                    break;

                default:
                    break;
            }
        }
        this.peerStatus = peerStatus;
    }

    /**
     * Because I HATE THIS SOUND THAT PLAYS EVERY TIME WE CONNECT THE ROBOT REEEEE
     */
    @Override public synchronized void updateWarningMessage(@Nullable String warningMessage)
    {
        if (warningMessage != null && !warningMessage.equals(this.warningMessage))
        {
            gotDisconnect = true;

            if (DEBUG) RobotLog.vv(SoundPlayer.TAG, "updateWarningMessage()");
//            playSound(soundWarning);
        }
        this.warningMessage = warningMessage;
    }
}
