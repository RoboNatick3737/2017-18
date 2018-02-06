package org.firstinspires.ftc.teamcode.vision.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;

import hankextensions.EnhancedOpMode;
import hankextensions.vision.vuforia.VuforiaCam;

@Autonomous(name="Check Vuforia", group= OpModeDisplayGroups.VISION_TESTING)
public class VuforiaTester extends EnhancedOpMode
{
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
            console.write(
                    "Vuforia template = " + RelicRecoveryVuMark.from(relicTemplate).toString()
            );
            flow.yield();
        }

    }
}
