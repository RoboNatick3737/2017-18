package org.firstinspires.ftc.teamcode.robot.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.HardwareBase;
import hankextensions.structs.Vector2D;

@Autonomous(name="Drive Testing", group=Constants.FINAL_BOT_EXPERIMENTATION)
public class DriveTesting extends HardwareBase
{
    @Override
    protected void onRunWithHardware() throws InterruptedException
    {
        waitForStart();

        swerveDrive.setDesiredMovement(Vector2D.polar(1, 0));

        flow.msPause(3000);

        swerveDrive.setDesiredMovement(Vector2D.polar(1, 180));

        flow.msPause(3000);

        swerveDrive.setDesiredMovement(Vector2D.polar(1, 90));

        flow.msPause(3000);

        swerveDrive.setDesiredMovement(Vector2D.polar(1, 270));

        flow.msPause(3000);
    }
}
