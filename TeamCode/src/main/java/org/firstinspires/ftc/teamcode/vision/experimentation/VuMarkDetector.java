package org.firstinspires.ftc.teamcode.vision.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
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
        vuforiaCam.start();
        VuforiaTrackable relicTemplate = vuforiaCam.getTrackables().get(0);
        vuforiaCam.getTrackables().activate();

        ProcessConsole console = log.newProcessConsole("Vuforia");

        while (true)
        {
            detectedVuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (detectedVuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                if (pose != null) {
//                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

//                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
//                    double tX = trans.get(0);
//                    double tY = trans.get(1);
//                    double tZ = trans.get(2);
//
//                    // Extract the rotational components of the target relative to the robot
//                    double rX = rot.firstAngle;
//                    double rY = rot.secondAngle;
//                    double rZ = rot.thirdAngle;


                    console.write(
                            "Detected: " + detectedVuMark.toString(),
                            "Rotation: <" + rot.firstAngle + ", " + rot.secondAngle + ", " + rot.thirdAngle + ">");
                }
                else
                {
                    console.write("Detected: " + detectedVuMark.toString());
                }
            }

            flow.yield();
        }

    }
}
