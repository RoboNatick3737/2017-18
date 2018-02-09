package org.firstinspires.ftc.teamcode.vision.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.vuforia.VuforiaCam;

@Autonomous(name="VuMark Detector", group= OpModeDisplayGroups.VISION_TESTING)
public class VuMarkDetector extends EnhancedOpMode
{
    private RelicRecoveryVuMark detectedVuMark = RelicRecoveryVuMark.UNKNOWN;
    public RelicRecoveryVuMark getDetectedVuMark()
    {
        return detectedVuMark;
    }

    @Override
    protected void onRun() throws InterruptedException
    {
        VuforiaCam vuforiaCam = new VuforiaCam();
        vuforiaCam.start(true);
        VuforiaTrackable relicTemplate = vuforiaCam.getTrackables().get(0);
        vuforiaCam.getTrackables().activate();

        ProcessConsole console = log.newProcessConsole("Vuforia");

        while (true)
        {
            detectedVuMark = RelicRecoveryVuMark.from(relicTemplate);

            console.write(
                    "Detected: " + detectedVuMark.toString()
            );
            flow.yield();
        }

    }
}
