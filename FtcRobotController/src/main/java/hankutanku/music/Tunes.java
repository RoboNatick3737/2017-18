package hankutanku.music;

import android.media.MediaPlayer;

import com.qualcomm.ftcrobotcontroller.R;

import dude.makiah.androidlib.logging.LoggingBase;

import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;

/**
 * NiFTMusic enables the easy playing of audio files which are placed in the FtcRobotController/res/raw folder.  This functionality is helpful for both debugging and showing the other teams that you've got swag.
 */
public class Tunes
{
    /**
     * The enum which encapsulates the res/raw folder (make sure to pre-register songs here!)
     */
    public enum Option
    {
        USSR_Anthem,
        LEFT_COL,
        CENTER_COL,
        RIGHT_COL,
        LEFT_RIGHT_JEWEL,
        RIGHT_LEFT_JEWEL
    }

    private static MediaPlayer mediaPlayer = null;

    /**
     * Calling play initializes the media player with the given app context and starts playing a song.
     *
     * @param choice the pre-registered enum option which represents the file in question.
     */
    public static void play (Option choice)
    {
        try
        {
            int selectedSong = com.qualcomm.ftcrobotcontroller.R.raw.ussranthem;

            //Add new mp3s here.
            switch (choice)
            {
                case USSR_Anthem:
                    selectedSong = com.qualcomm.ftcrobotcontroller.R.raw.ussranthem;
                    break;

                case LEFT_COL:
                    selectedSong = R.raw.leftcol;
                    break;

                case CENTER_COL:
                    selectedSong = R.raw.centercol;
                    break;

                case RIGHT_COL:
                    selectedSong = R.raw.rightcol;
                    break;

                case LEFT_RIGHT_JEWEL:
                    selectedSong = R.raw.leftrightjewel;
                    break;

                case RIGHT_LEFT_JEWEL:
                    selectedSong = R.raw.rightleftjewel;
                    break;
            }

            mediaPlayer = MediaPlayer.create (EnhancedOpMode.instance.hardwareMap.appContext, selectedSong);
            mediaPlayer.start ();
            mediaPlayer.setOnCompletionListener (new MediaPlayer.OnCompletionListener ()
            {
                public void onCompletion (MediaPlayer mediaPlayer1)
                {
                    mediaPlayer1.release ();
                }
            });

            LoggingBase.instance.lines("Playing " + choice.toString ());

            EnhancedOpMode.instance.flow.pause (new TimeMeasure(TimeMeasure.Units.SECONDS, 1)); //Give the MediaPlayer some time to initialize, and register that a song is being played.
        } catch (InterruptedException e)
        {/**/} //Exit immediately.
        catch (Exception e)
        {
            //#FREECICE
            EnhancedOpMode.instance.log.lines("Music error: " + e.getMessage ());
        }
    }

    /**
     * Calling silence() stops the currently playing media player, if it is playing.
     */
    public static void silence()
    {
        if (mediaPlayer != null)
        {
            if (mediaPlayer.isPlaying ())
                mediaPlayer.stop (); //stopEasyTask playing
            mediaPlayer.release (); //prevent resource allocation
            mediaPlayer = null; //nullify the reference.
        }
    }

    public static boolean playing ()
    {
        return mediaPlayer != null;
    }
}
